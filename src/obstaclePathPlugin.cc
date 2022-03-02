#include <uav_simulator/obstaclePathPlugin.hh>

namespace gazebo
{
  void DynamicObstacle::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){
      // Store the pointer to the model
      this->model = _parent;
      this->sdf = _sdf;

      // Read paramters in sdf file
      if (this->sdf->HasElement("velocity")){
        this->velocity = _sdf->Get<double>("velocity");
      }
      else{
        this->velocity = 1.0;
      }

      if (this->sdf->HasElement("angular_velocity")){
        this->angularVelocity = _sdf->Get<double>("angular_velocity");
      }
      else{
        this->angularVelocity = 0.8;
      }

      // read path:
      this->path.clear();
      if (this->sdf->HasElement("path")){
        sdf::ElementPtr waypointElem = _sdf->GetElement("path")->GetElement("waypoint");
        while (waypointElem){
          ignition::math::Vector3d wp = waypointElem->Get<ignition::math::Vector3d>();
          this->path.push_back(wp);
          waypointElem = waypointElem->GetNextElement("waypoint");
        }
      }
      this->path.push_back(this->path[0]); // form a loop
      


      // modify to the path with angle
      this->pathWithAngle.clear();
      double yawCurr;
      double yawLast;
      for (int i=0; i<this->path.size(); ++i){
        if (i == 0){
          double xCurr, yCurr, zCurr, yawStart;
          double xNext, yNext, zNext;

          xCurr = this->path[i].X();
          yCurr = this->path[i].Y();
          zCurr = this->path[i].Z();

          xNext = this->path[i+1].X();
          yNext = this->path[i+1].Y();
          zNext = this->path[i+1].Z();
          
          yawStart = atan2(yNext-yCurr, xNext-xCurr);
          std::vector<double> pose {xCurr, yCurr, zCurr, yawStart};
          this->pathWithAngle.push_back(pose);
          yawLast = yawStart;
        }
        else{
          // if not zero, we need to add two poses for different yaw
          double xCurr, yCurr, zCurr;
          double xNext, yNext, zNext;
          double xPrev, yPrev, zPrev;
          // double xHat, yHat, zHat; // for bad gazebo

          xCurr = this->path[i].X();
          yCurr = this->path[i].Y();
          zCurr = this->path[i].Z();



          if (i+1 < this->path.size()){
            xNext = this->path[i+1].X();
            yNext = this->path[i+1].Y();
            zNext = this->path[i+1].Z();
          }
          else{
            xNext = this->path[1].X();
            yNext = this->path[1].Y();
            zNext = this->path[1].Z();
          }

          xPrev = this->path[i-1].X();
          yPrev = this->path[i-1].Y();
          zPrev = this->path[i-1].Z();

          yawCurr = atan2(yNext-yCurr, xNext-xCurr);

          // double distance = sqrt(pow(xCurr-xPrev, 2) + pow(yCurr-yPrev, 2) + pow(zCurr-zPrev, 2));
          
          // xHat = xPrev + (distance - 2.0)/distance * (xCurr - xPrev);
          // yHat = yPrev + (distance - 2.0)/distance * (yCurr - yPrev);
          // zHat = zPrev + (distance - 2.0)/distance * (zCurr - zPrev);


          // adding first point:
          std::vector<double> pose1 {xCurr, yCurr, zCurr, yawLast};
          // std::vector<double> pose1 {xHat, yHat, zHat, yawLast};
          this->pathWithAngle.push_back(pose1);
          
          // ading second point:
          std::vector<double> pose2 {xCurr, yCurr, zCurr, yawCurr};
          this->pathWithAngle.push_back(pose2);

          yawLast = yawCurr;
        }
      }


      // calculate total time
      this->timeKnot.clear();
      double totalTime = 0.0;
      this->timeKnot.push_back(totalTime);
      for (int i=0; i<this->pathWithAngle.size()-1; ++i){
        std::vector poseCurr = this->pathWithAngle[i];
        std::vector poseNext = this->pathWithAngle[i+1];

        if (i % 2 == 0){ // forward motion
          double xCurr, yCurr, zCurr;
          double xNext, yNext, zNext;

          xCurr = poseCurr[0];
          yCurr = poseCurr[1];
          zCurr = poseCurr[2];

          xNext = poseNext[0];
          yNext = poseNext[1];
          zNext = poseNext[2];

          double distance = sqrt(pow(xNext-xCurr, 2) + pow(yNext-yCurr, 2) + pow(zNext-zCurr, 2));
          int duration = distance/this->velocity;
          totalTime += duration;
          this->timeKnot.push_back(totalTime);
        }
        else{ // rotation
          double yawCurr, yawNext;
          yawCurr = poseCurr[3];
          yawNext = poseNext[3];
          double angleABSDiff = std::abs(atan2(sin(yawNext-yawCurr), cos(yawNext-yawCurr)));
          double duration = angleABSDiff/this->angularVelocity;
          totalTime += duration;
          this->timeKnot.push_back(totalTime);
        }
      }



      gazebo::common::PoseAnimationPtr anim(new gazebo::common::PoseAnimation("obstaclePathLoop", totalTime, true));
      gazebo::common::PoseKeyFrame *key;
      for (int i=0; i<this->pathWithAngle.size(); ++i){
        double t = this->timeKnot[i];
        std::vector<double> pose = this->pathWithAngle[i];
        double x = pose[0];
        double y = pose[1];
        double z = pose[2];
        double yaw = pose[3];
        key = anim->CreateKeyFrame(t);
        key->Translation(ignition::math::Vector3d(x, y, z));
        key->Rotation(ignition::math::Quaterniond(0, 0, yaw));
        gzdbg << "t: " << t << ", pose: " << "(" << x << " " << y << " " << z << " "<<  yaw << ")" << std::endl;  
      }

        // // create the animation
        // gazebo::common::PoseAnimationPtr anim(
        //       // name the animation "test",
        //       // make it last 10 seconds,
        //       // and set it on a repeat loop
        //       new gazebo::common::PoseAnimation("test", 10.0, true));

        // gazebo::common::PoseKeyFrame *key;

        // // set starting location of the box
        // key = anim->CreateKeyFrame(0);
        // key->Translation(ignition::math::Vector3d(0, 0, 0));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        // // set waypoint location after 2 seconds
        // key = anim->CreateKeyFrame(2.0);
        // key->Translation(ignition::math::Vector3d(-5, -5, 0));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 1.5707));


        // key = anim->CreateKeyFrame(4.0);
        // key->Translation(ignition::math::Vector3d(10, 20, 0));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 1.5707));


        // key = anim->CreateKeyFrame(6.0);
        // key->Translation(ignition::math::Vector3d(-10, 20, 0));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 1.5707));


        // key = anim->CreateKeyFrame(8.0);
        // key->Translation(ignition::math::Vector3d(10, -20, 0));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 1.5707));

        // // set final location equal to starting location
        // key = anim->CreateKeyFrame(10);
        // key->Translation(ignition::math::Vector3d(0, 0, 0));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 0));
        // set starting location of the box
        // key = anim->CreateKeyFrame(0);
        // key->Translation(ignition::math::Vector3d(0, 0, 0));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        // set waypoint location after 2 seconds
        // key = anim->CreateKeyFrame(0.0);
        // key->Translation(ignition::math::Vector3d(-5, -5, 0.1));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 1.5707));


        // key = anim->CreateKeyFrame(2);
        // key->Translation(ignition::math::Vector3d(-5, -3, 0.1));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 1.5707));


        // key = anim->CreateKeyFrame(20);
        // key->Translation(ignition::math::Vector3d(5, 5, 0.1));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 1.5707));


        // key = anim->CreateKeyFrame(30);
        // key->Translation(ignition::math::Vector3d(5, -5, 0.1));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 1.5707));

        // // set final location equal to starting location
        // key = anim->CreateKeyFrame(40);
        // key->Translation(ignition::math::Vector3d(-5, -5, 0.1));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 0));
        // set the animation
        _parent->SetAnimation(anim);
    }
}
