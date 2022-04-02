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

      if (this->sdf->HasElement("orientation")){
        this->orientation = _sdf->Get<bool>("orientation");
      }
      else{
        this->orientation = true;
      }

      if (this->orientation){
        if (this->sdf->HasElement("angular_velocity")){
          this->angularVelocity = _sdf->Get<double>("angular_velocity");
        }
        else{
          this->angularVelocity = 0.8;
        }
      }

      if (this->sdf->HasElement("loop")){
        this->loop = _sdf->Get<bool>("loop");
      }
      else{
        this->loop = false;
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

      
      if (this->loop){
        this->path.push_back(this->path[0]); // form a loop
      }
      else{
        // back and forth
        std::vector<ignition::math::Vector3d> temp = this->path;
        for (int i=temp.size()-2; i>=0; --i){
          this->path.push_back(temp[i]);
        }
      }

      if (this->orientation){
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

            // adding first point:
            std::vector<double> pose1 {xCurr, yCurr, zCurr, yawLast};
            // std::vector<double> pose1 {xCurr, yCurr, zCurr, yawCurr};
            // std::vector<double> pose1 {xHat, yHat, zHat, yawLast};
            this->pathWithAngle.push_back(pose1);

            // ading last point:
            std::vector<double> pose2 {xCurr, yCurr, zCurr, yawCurr};
            this->pathWithAngle.push_back(pose2);

            yawLast = yawCurr;
          }
        }
      }
      else{
        this->pathWithAngle.clear();
        for (int i=0; i<this->path.size(); ++i){
          ignition::math::Vector3d wp = this->path[i];
          std::vector<double> pose {wp.X(), wp.Y(), wp.Z(), 0};
          this->pathWithAngle.push_back(pose);
        }
      }


      // calculate total time
      this->timeKnot.clear();
      double totalTime = 0.0;
      this->timeKnot.push_back(totalTime);
      for (int i=0; i<this->pathWithAngle.size()-1; ++i){
        std::vector<double> poseCurr = this->pathWithAngle[i];
        std::vector<double> poseNext = this->pathWithAngle[i+1];

        bool rotation = ((poseCurr[0] == poseNext[0]) and (poseCurr[1] == poseNext[1]) and (poseCurr[2] == poseNext[2]));

        if (not rotation){ // forward motion
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
        // double yaw = 1.5707;
        key = anim->CreateKeyFrame(t);
        key->Translation(ignition::math::Vector3d(x, y, z));
        key->Rotation(ignition::math::Quaterniond(0, 0, yaw));
        gzdbg << "t: " << t << ", pose: " << "(" << x << " " << y << " " << z << " "<<  yaw << ")" << std::endl;  
      }

        // // set the animation
        _parent->SetAnimation(anim);
    }

    std::vector<double>& DynamicObstacle::interpolateAngle(double start, double end, double dx){
      static std::vector<double> interpolation;
      double angleDiff = end - start;
      double angleDiffABS = std::abs(angleDiff);

      if (angleDiff >= 0 and angleDiffABS <= M_PI){
        for (double mid=start+dx; mid<end; mid+=dx){
          interpolation.push_back(mid);
        }
      }
      else if (angleDiff >= 0 and angleDiffABS > M_PI){
        // minus unitl -PI
        double mid = start-dx;
        while (mid >= -M_PI){
          interpolation.push_back(mid);
          mid -= dx;
        }

        // minus from PI to end
        mid = M_PI;
        while (mid > end){
          interpolation.push_back(mid);
          mid -= dx;
        }
      }
      else if (angleDiff < 0 and angleDiffABS <= M_PI){
        for (double mid=start-dx; mid>end; mid-=dx){
          interpolation.push_back(mid);
        }
      }
      else if (angleDiff < 0 and angleDiffABS > M_PI){
        // plus until PI
        double mid = start+dx;
        while (mid <= M_PI){
          interpolation.push_back(mid);
          mid += dx;
        }

        // plus from -PI to end
        mid = -M_PI;
        while (mid < end){
          interpolation.push_back(mid);
          mid += dx;
        }
      }
      return interpolation;
    }
}
