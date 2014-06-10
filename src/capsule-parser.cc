#include "capsule-parser.hh"
#include "precomputation-utils.hh"

namespace hpp
{
  namespace corbaserver
  {
    namespace motionprior
    {
      namespace capsules
      {
        std::vector<ProjectedCapsulePoint> computeConvexHullFromProjectedCapsulePoints (const std::vector<ProjectedCapsulePoint> &capsVec) 
        {
          return ::hpp::corbaserver::motionprior::convexhull::convex_hull(capsVec);
        }

        bool isSubsetOf(const std::vector<ProjectedCapsulePoint> &lhs, const std::vector<ProjectedCapsulePoint> &rhs) 
        {
          for(uint i=0 ; i < lhs.size() ; ++i ){
            if(!lhs.at(i).isInsideConvexHull(rhs)){
              return false;
            }
          }
          return true;
        }
        bool isSmallerVolume(const std::vector<ProjectedCapsulePoint> &lhs, const std::vector<ProjectedCapsulePoint> &rhs) 
        {
          double lv = getVolume(lhs);
          double rv = getVolume(rhs);
          return lv <= rv;
        }

        double getVolume(const std::vector<ProjectedCapsulePoint> &pts){
          //assume that hull are points on a convex hull, which are sorted
          //counter-clockwise.
          using namespace Eigen;
          Vector2f x1(pts.at(0).y, pts.at(0).z);
          double volume = 0;
          //compute volume by iterating over all embeded triangles
          for(int i=0; i<pts.size()-2; i++){
            Vector2f x2(pts.at(i+1).y, pts.at(i+1).z);
            Vector2f x3(pts.at(i+2).y, pts.at(i+2).z);
            Vector2f v12 = x2-x1;
            Vector2f v13 = x3-x1;
            double b = v12.norm();
            Vector2f h_vec = v13 - v13.dot(v12)*v12;
            double h = h_vec.norm();
            double d = 0.5*b*h;
            volume += d;
          }
          return volume;
        }

        bool ProjectedCapsulePoint::isInsideConvexHull(const std::vector<ProjectedCapsulePoint> &ptsOnCvxHullCounterClockwise) const
        {
          for(uint i=0 ; i < ptsOnCvxHullCounterClockwise.size() ; ++i ){
            ProjectedCapsulePoint v = ptsOnCvxHullCounterClockwise.at(i);
            ProjectedCapsulePoint vn;
        
            if(i==ptsOnCvxHullCounterClockwise.size()-1){
              vn = ptsOnCvxHullCounterClockwise.at(0);
            }else{
              vn = ptsOnCvxHullCounterClockwise.at(i+1);
            }
            double ey = vn.y - v.y;
            double ez = vn.z - v.z;
        
            double dy = this->y - v.y;
            double dz = this->z - v.z;
        
            double dist = ey*dz - ez*dy;
            //hppDout(notice, "cvx v=[" << v.y << " " << v.z << "] vn=[" << vn.y << "," << vn.z << "]" 
                            //<< " pt [" << this->y << "," << this->z << "]" << " dist " << dist );
            if(dist < -0.05){ //allow small uncertainty
              return false;
            }
          } 
          return true;
        }

        hpp::floatSeq* capsulePointsToFloatSeq (const std::vector<ProjectedCapsulePoint> &capsVector) 
           throw (hpp::Error)
        {
          try {
            hpp::floatSeq* capsFloatSeq = new hpp::floatSeq;
            capsFloatSeq->length (3*capsVector.size());
            int ctr = 0;
            for(uint i=0;i<capsVector.size();i++){
              (*capsFloatSeq)[ctr++] = 0;
              (*capsFloatSeq)[ctr++] = capsVector.at(i).y;
              (*capsFloatSeq)[ctr++] = capsVector.at(i).z;
            }
            return capsFloatSeq;
          } catch (const std::exception& exc) {
            hppDout (error, exc.what ());
            throw hpp::Error (exc.what ());
          }
        }
        hpp::floatSeq* capsulePointsToFloatSeq (const std::vector<CapsulePoint> &capsVector) 
           throw (hpp::Error)
        {
          try {
            hpp::floatSeq* capsFloatSeq = new hpp::floatSeq;
            capsFloatSeq->length (3*capsVector.size());
            int ctr = 0;
            for(uint i=0;i<capsVector.size();i++){
              (*capsFloatSeq)[ctr++] = capsVector.at(i).x;
              (*capsFloatSeq)[ctr++] = capsVector.at(i).y;
              (*capsFloatSeq)[ctr++] = capsVector.at(i).z;
            }
            return capsFloatSeq;
          } catch (const std::exception& exc) {
            hppDout (error, exc.what ());
            throw hpp::Error (exc.what ());
          }
        }

        std::vector<ProjectedCapsulePoint> projectCapsulePointsOnYZPlane (const std::vector<CapsulePoint> &capsVec) 
        {
          std::vector<ProjectedCapsulePoint> projCapsulePointsYZPlane;
          for(uint i=0; i<capsVec.size(); i++){
            ProjectedCapsulePoint p;
            p.y = capsVec.at(i).y;
            p.z = capsVec.at(i).z;
            p.J = capsVec.at(i).J;
            projCapsulePointsYZPlane.push_back(p);

            // orthogonal projection by just removing the x-component of the
            // point.
            // we still need to compute the outer points, which are located at
            // radius from the capsule point on the YZ plane. We will compute only
            // a finite number of outer points, which is equal to an approximation
            // of the circle by an inner polygon. Change the t_step size to get
            // a better approximation.
            double t_step = M_PI/6;
            double yP = capsVec.at(i).y;
            double zP = capsVec.at(i).z;
            double radius = capsVec.at(i).radius;
            for(double theta = 0; theta <= 2*M_PI; theta+=t_step){
              ProjectedCapsulePoint outerPoint;
              outerPoint.y = cos(theta)*radius + yP;
              outerPoint.z = sin(theta)*radius + zP;
              outerPoint.J = capsVec.at(i).J;
              projCapsulePointsYZPlane.push_back(outerPoint);
            }
          }
          return projCapsulePointsYZPlane;
        }

        std::vector<CapsulePoint> parseCapsulePoints (DevicePtr_t robot) throw (hpp::Error)
        {
          try {
            using namespace std;
            using namespace fcl;
            std::vector<CapsulePoint> capsuleVec;
            JointVector_t jointVec = robot->getJointVector();
            std::stringstream stream; //DEBUG stream

            for(uint i=0; i<jointVec.size(); i++){
              //-----------------------------------------------
              JointPtr_t joint = jointVec.at(i);
              const hpp::model::JointJacobian_t jjacobian = joint->jacobian();
              //-----------------------------------------------
              BodyPtr_t body = joint->linkedBody();
              if (!body) {
                stream << "JOINT has no body" << endl;
                continue;
              }
              stream << "JOINT " << joint->name () << endl;
              stream << "BODY " << body->name () << endl;
              //-----------------------------------------------
              ObjectVector_t objects = body->innerObjects (model::COLLISION);
              if (objects.size() > 0) {
                std::size_t nbObjects = objects.size();
                for (std::size_t iObject=0; iObject < nbObjects; iObject++) {
                  //-----------------------------------------------
                  CollisionObjectPtr_t object = objects[iObject];
                  std::string geometryName = object->name();
                  stream << "OBJECT " << geometryName << endl;
                  //-----------------------------------------------
                  fcl::CollisionObjectPtr_t fco = object->fcl();
                  //-----------------------------------------------
                  const fcl::NODE_TYPE nodeType = fco->getNodeType();
                  const fcl::OBJECT_TYPE objectType = fco->getObjectType();
                  //safety check the right node type:
                  // geometry -- capsules
                  if(objectType != OT_GEOM){
                    hppDout(error, "OBJECT_TYPE " << objectType << " not handled by function");
                  }
                  if(nodeType != GEOM_CAPSULE){
                    hppDout(error, "NODE_TYPE " << nodeType << " not handled by function");
                  }
                  const fcl::Capsule *capsule = static_cast<const fcl::Capsule*>(fco->getCollisionGeometry());

                  double length = capsule->lz;
                  double radius = capsule->radius;
                  //-----------------------------------------------
                  //compute the outer points of the
                  //capsule: ( x1 -----o----- x2 )
                  //here, o depicts the center of the
                  //capsule, which is given by aabb_center
                  //x1,x2 are the center points of the top 
                  //and bottom disc of the
                  //cylinder, respectively.

                  fcl::Vec3f center = capsule->aabb_center;
                  fcl::Transform3f T = fco->getTransform();

                  //create points all along the axis of
                  //the capsule and project them under the
                  //given transformation
                  double l = -length/2;
                  while(l <= length/2){
                    fcl::Vec3f x(0,0,l);
                    fcl::Transform3f Tx;
                    Tx.setTranslation(x);
                    Tx = T*Tx;
                    x = Tx.getTranslation();
                    CapsulePoint p;
                    p.x = x[0];
                    p.y = x[1];
                    p.z = x[2];
                    p.radius = radius;
                    p.length = length;
                    p.J = jjacobian;
                    capsuleVec.push_back(p);
                    l += length;
                  }
                  //-----------------------------------------------
                  //fcl::AABB aabb = fco->getAABB();
                  //stream << aabb.width() << " " << aabb.height() << endl;
                }//iterate inner objects
              }
            }//iterate joints

            //IF DEBUG
            //hppDout(notice, stream.str());
            return capsuleVec;

          } catch (const std::exception& exc) {
            hppDout (error, exc.what ());
            throw hpp::Error (exc.what ());
          }
        }
      } // end of namespace capsules
    } // end of namespace motionprior
  } // end of namespace corbaServer.
} // end of namespace hpp.


