//
// Created by tpcw on 27-05-20.
//

#include "Trajectories.h"

Trajectories::Trajectories(int n) : Trajectories_numb(n), noiseSTD(0.01), switch_ID1(0), switch_ID2(0), switch_ID3(0),
delay1(0), delay2(0), delay3(0), labdaFP(0.2), newArrivalTimeFP(0) {
    initialize();
}

void Trajectories::initialize() {
    // Set node handle
    ros::NodeHandle gen_ev;

    // Publishers
    world_evidence_publisher_ = gen_ev.advertise<wire_msgs::WorldEvidence>("/world_evidence", 100);
    OOS_publisher_ = gen_ev.advertise<std_msgs::Float32MultiArray>("/OOS_range", 100);

    time_last_cycle_ = ros::Time::now();
    return;
}

void Trajectories::publish() {
    time_now = ros::Time::now();
    dt = (time_now - time_last_cycle_).toSec();

    // Create world evidence message
    wire_msgs::WorldEvidence world_evidence;

	// Set header
	world_evidence.header.stamp = ros::Time::now();
	world_evidence.header.frame_id = "/map";

	// Define situation
    if(Trajectories_numb == 1) {
        Situation1(world_evidence); // Path people
    } else if(Trajectories_numb == 2) {
        Situation2(world_evidence); // Path people
    } else if(Trajectories_numb == 3) {
        Situation3(world_evidence); // Path people
    } else if(Trajectories_numb == 4) {
        Situation4(world_evidence); // Path people
    } else if(Trajectories_numb == 5) {
        Situation5(world_evidence); // Path people
    } else if(Trajectories_numb == 6) {
        Situation6(world_evidence); // Path people
    } else if(Trajectories_numb == 7) {
        Situation7(world_evidence); // Path people
    } else {
        ROS_ERROR("The situation does not exist");
    }

//    FalsePositives(world_evidence); // False positives

	// Publish results
	world_evidence_publisher_.publish(world_evidence);
    time_last_cycle_ = time_now;

	ROS_INFO("Published world evidence with size %d", world_evidence.object_evidence.size());
    return;
}

void Trajectories::FalsePositives(wire_msgs::WorldEvidence& world_evidence) {
    // Random sets
    std::exponential_distribution<double> exp(labdaFP) ; // Poisson arrival time for false positives
    std::uniform_real_distribution<> disx(-1.0, 3.0)   ; // Uniform distribution over the room
    std::uniform_real_distribution<> disy(-2.0, 2.0)   ; // Uniform distribution over the room

    // Generate false positives
    if (time_now > newArrivalTimeFP){
        x_personFP = disx(det_gen);
        y_personFP = disy(det_gen);
        if((y_personFP < -0.55 || y_personFP > 0.55) && x_personFP < 1.3) {
            addEvidence(world_evidence, x_personFP, y_personFP, 1, "person", "");
            newArrivalTimeFP = time_now + ros::Duration(exp(det_gen));
        }
    }
    return;
}

void Trajectories::addEvidence(wire_msgs::WorldEvidence& world_evidence, double x, double y, double z, const std::string& class_label, const std::string& color) {
    wire_msgs::ObjectEvidence obj_evidence;

    // Set the continuous position property
    wire_msgs::Property posProp;
    posProp.attribute = "position";

    // Set position (x,y,z), set the covariance matrix as 0.005*identity_matrix
    pbl::PDFtoMsg(pbl::Gaussian(pbl::Vector3(x, y, z), pbl::Matrix3(0.0005, 0.0005, 0.0005)), posProp.pdf);
    obj_evidence.properties.push_back(posProp);

    // Set the discrete class label property
    wire_msgs::Property classProp;
    classProp.attribute = "class_label";
    pbl::PMF classPMF;

    // Probability of the class label is 0.7
    classPMF.setProbability(class_label, 0.7);
    pbl::PDFtoMsg(classPMF, classProp.pdf);
    obj_evidence.properties.push_back(classProp);

    // Set the discrete color property
    wire_msgs::Property colorProp;
    colorProp.attribute = "color";
    pbl::PMF colorPMF;

    // The probability of the detected color is 0.1
    colorPMF.setProbability(color, 0.1);
    pbl::PDFtoMsg(colorPMF, colorProp.pdf);
    obj_evidence.properties.push_back(colorProp);

    // Add all properties to the array
    world_evidence.object_evidence.push_back(obj_evidence);
    return;
}

/// SITUATIONS ///
/// SITUATION 1: Single person walks behind closet continuously
void Trajectories::Situation1(wire_msgs::WorldEvidence& world_evidence){
    // Define velocity person
    double vel_person = 0.5;

    // Random sets
    std::normal_distribution<> detNoise{0,noiseSTD}    ; // Gaussian noise mean = 0, std = noiseSTD

    switch(switch_ID1){
        case 0: // Wait for rviz to start up
            x_person1 = 0   + detNoise(det_gen);
            y_person1 = 1.5 + detNoise(det_gen);
            delay1 += dt;
            if(delay1 > 4){
                delay1 = 0;
                switch_ID1 = 1;
            }
            break;
        case 1: // Move in positive x-direction
            x_person1 += dt * vel_person + detNoise(det_gen);
            y_person1 += 0               + detNoise(det_gen);
            if(x_person1 > 1.5){
                switch_ID1 = 2;
            }
            break;
        case 2: // Turn
            x_person1 += std::cos(delay1 * 0.5 * M_PI) *  dt * vel_person + detNoise(det_gen);
            y_person1 += std::sin(delay1 * 0.5 * M_PI) * -dt * vel_person + detNoise(det_gen);
            delay1 += dt;
            if(delay1 > 1){
                delay1 = 0;
                switch_ID1 = 3;
            }
            break;
        case 3: // Move in negative y-direction
            x_person1 += 0;
            y_person1 += -dt * vel_person;
            if(y_person1 < -1){
                switch_ID1 = 4;
            }
            break;
        case 4: // Turn
            x_person1 += -std::sin(delay1 * 0.5 * M_PI) *  dt * vel_person + detNoise(det_gen);
            y_person1 +=  std::cos(delay1 * 0.5 * M_PI) * -dt * vel_person + detNoise(det_gen);
            delay1 += dt;
            if(delay1 > 1){
                delay1 = 0;
                switch_ID1 = 5;
            }
            break;
        case 5: // Move in negative x-direction
            x_person1 += -dt * vel_person + detNoise(det_gen);
            y_person1 += 0                + detNoise(det_gen);
            if(x_person1 < 0){
                switch_ID1 = 6;
            }
            break;
        case 6: // End position
            x_person1 += 0 + detNoise(det_gen);
            y_person1 += 0 + detNoise(det_gen);
            break;
    }

    // Add evidence person
    if(y_person1 < -0.55 || y_person1 > 0.55) {
        addEvidence(world_evidence, x_person1, y_person1, 1, "person", "red");
    }

    // Send info about OOS area
    OOS_range_.data.clear();
    OOS_range_.data.push_back( 1.6); // OOS_min_x
    OOS_range_.data.push_back( 3.0); // OOS_max_x
    OOS_range_.data.push_back(-0.6); // OOS_min_y
    OOS_range_.data.push_back( 0.6); // OOS_max_y
    OOS_publisher_.publish(OOS_range_);

    return;
}

/// SITUATION 2: Single person walks behind closet and walks back
void Trajectories::Situation2(wire_msgs::WorldEvidence& world_evidence){
    // Define velocity person
    double vel_person = 0.5;

    // Random sets
    std::normal_distribution<> detNoise{0,noiseSTD}    ; // Gaussian noise mean = 0, std = noiseSTD

    switch(switch_ID1){
        case 0: // Wait for rviz to start up
            x_person1 = 0   + detNoise(det_gen);
            y_person1 = 1.5 + detNoise(det_gen);
            delay1 += dt;
            if(delay1 > 4){
                delay1 = 0;
                switch_ID1 = 1;
            }
            break;
        case 1: // Move in positive x-direction
            x_person1 += dt * vel_person + detNoise(det_gen);
            y_person1 += 0               + detNoise(det_gen);
            if(x_person1 > 1.5){
                switch_ID1 = 2;
            }
            break;
        case 2: // Turn
            x_person1 += std::cos(delay1 * 0.5 * M_PI) *  dt * vel_person + detNoise(det_gen);
            y_person1 += std::sin(delay1 * 0.5 * M_PI) * -dt * vel_person + detNoise(det_gen);
            delay1 += dt;
            if(delay1 > 1){
                delay1 = 0;
                switch_ID1 = 3;
            }
            break;
        case 3: // Move in negative y-direction
            x_person1 += 0;
            y_person1 += -dt * vel_person;
            if(y_person1 < 0){
                switch_ID1 = 4;
            }
            break;
        case 4: // Move in positive y-direction
            x_person1 += 0;
            y_person1 += dt * vel_person;
            if(y_person1 > 1){
                switch_ID1 = 5;
            }
            break;
        case 5: // Turn
            x_person1 += -std::sin(delay1 * 0.5 * M_PI) *  dt * vel_person + detNoise(det_gen);
            y_person1 += -std::cos(delay1 * 0.5 * M_PI) * -dt * vel_person + detNoise(det_gen);
            delay1 += dt;
            if(delay1 > 1){
                delay1 = 0;
                switch_ID1 = 6;
            }
            break;
        case 6: // Move in negative x-direction
            x_person1 += -dt * vel_person + detNoise(det_gen);
            y_person1 += 0                + detNoise(det_gen);
            if(x_person1 < 0){
                switch_ID1 = 7;
            }
            break;
        case 7: // End position
            x_person1 += 0 + detNoise(det_gen);
            y_person1 += 0 + detNoise(det_gen);
            break;
    }

    // Add evidence person
    if(y_person1 < -0.55 || y_person1 > 0.55) {
        addEvidence(world_evidence, x_person1, y_person1, 1, "person", "red");
    }

    // Send info about OOS area
    OOS_range_.data.clear();
    OOS_range_.data.push_back( 1.6); // OOS_min_x
    OOS_range_.data.push_back( 3.0); // OOS_max_x
    OOS_range_.data.push_back(-0.6); // OOS_min_y
    OOS_range_.data.push_back( 0.6); // OOS_max_y
    OOS_publisher_.publish(OOS_range_);

    return;
}

/// SITUATION 3: Single person walks behind closet, new person appears
void Trajectories::Situation3(wire_msgs::WorldEvidence& world_evidence){
    // Define velocity person
    double vel_person = 0.5;

    // Random sets
    std::normal_distribution<> detNoise{0,noiseSTD}    ; // Gaussian noise mean = 0, std = noiseSTD

    switch(switch_ID1){
        case 0: // Wait for rviz to start up
            x_person1 = 0   + detNoise(det_gen);
            y_person1 = 1.5 + detNoise(det_gen);
            delay1 += dt;
            if(delay1 > 4){
                delay1 = 0;
                switch_ID1 = 1;
            }
            break;
        case 1: // Move in positive x-direction
            x_person1 += dt * vel_person + detNoise(det_gen);
            y_person1 += 0               + detNoise(det_gen);
            if(x_person1 > 1.5){
                switch_ID1 = 2;
            }
            break;
        case 2: // Turn
            x_person1 += std::cos(delay1 * 0.5 * M_PI) *  dt * vel_person + detNoise(det_gen);
            y_person1 += std::sin(delay1 * 0.5 * M_PI) * -dt * vel_person + detNoise(det_gen);
            delay1 += dt;
            if(delay1 > 1){
                delay1 = 0;
                switch_ID1 = 3;
            }
            break;
        case 3: // Move in negative y-direction
            x_person1 += 0;
            y_person1 += -dt * vel_person;
            if(y_person1 < -1){
                switch_ID1 = 4;
            }
            break;
        case 4: // Turn
            x_person1 += -std::sin(delay1 * 0.5 * M_PI) *  dt * vel_person + detNoise(det_gen);
            y_person1 +=  std::cos(delay1 * 0.5 * M_PI) * -dt * vel_person + detNoise(det_gen);
            delay1 += dt;
            if(delay1 > 1){
                delay1 = 0;
                switch_ID1 = 5;
            }
            break;
        case 5: // Move in negative x-direction
            x_person1 += -dt * vel_person + detNoise(det_gen);
            y_person1 += 0                + detNoise(det_gen);
            if(x_person1 < 0){
                switch_ID1 = 6;
            }
            break;
        case 6: // End position
            x_person1 += 0 + detNoise(det_gen);
            y_person1 += 0 + detNoise(det_gen);
            break;
    }

    // Add evidence person
    if(y_person1 > 0.55) {
        addEvidence(world_evidence, x_person1, y_person1, 1, "person", "red");
    } else if (y_person1 < -0.55) {
        addEvidence(world_evidence, x_person1, y_person1, 1, "person", "green");
    }

    // Send info about OOS area
    OOS_range_.data.clear();
    OOS_range_.data.push_back( 1.6); // OOS_min_x
    OOS_range_.data.push_back( 3.0); // OOS_max_x
    OOS_range_.data.push_back(-0.6); // OOS_min_y
    OOS_range_.data.push_back( 0.6); // OOS_max_y
    OOS_publisher_.publish(OOS_range_);

    return;
}

/// SITUATION 4: Two persons are occluded at the same time, no sensor to distinguish
void Trajectories::Situation4(wire_msgs::WorldEvidence& world_evidence){
    // Define velocity person
    double vel_person = 0.5;

    // Random sets
    std::normal_distribution<> detNoise{0,noiseSTD}    ; // Gaussian noise mean = 0, std = noiseSTD

    // Person 1
    switch(switch_ID1){
        case 0: // Wait for rviz to start up
            x_person1 = 0   + detNoise(det_gen);
            y_person1 = 1.5 + detNoise(det_gen);
            delay1 += dt;
            if(delay1 > 4){
                delay1 = 0;
                switch_ID1 = 1;
            }
            break;
        case 1: // Move in positive x-direction
            x_person1 += dt * vel_person + detNoise(det_gen);
            y_person1 += 0               + detNoise(det_gen);
            if(x_person1 > 1.5){
                switch_ID1 = 2;
            }
            break;
        case 2: // Turn
            x_person1 += std::cos(delay1 * 0.5 * M_PI) *  dt * vel_person + detNoise(det_gen);
            y_person1 += std::sin(delay1 * 0.5 * M_PI) * -dt * vel_person + detNoise(det_gen);
            delay1 += dt;
            if(delay1 > 1){
                delay1 = 0;
                switch_ID1 = 3;
            }
            break;
        case 3: // Move in negative y-direction
            x_person1 += 0;
            y_person1 += -dt * vel_person;
            if(y_person1 < -1){
                switch_ID1 = 4;
            }
            break;
        case 4: // Turn
            x_person1 += -std::sin(delay1 * 0.5 * M_PI) *  dt * vel_person + detNoise(det_gen);
            y_person1 +=  std::cos(delay1 * 0.5 * M_PI) * -dt * vel_person + detNoise(det_gen);
            delay1 += dt;
            if(delay1 > 1){
                delay1 = 0;
                switch_ID1 = 5;
            }
            break;
        case 5: // Move in negative x-direction
            x_person1 += -dt * vel_person + detNoise(det_gen);
            y_person1 += 0                + detNoise(det_gen);
            if(x_person1 < 0){
                switch_ID1 = 6;
            }
            break;
        case 6: // End position
            x_person1 += 0 + detNoise(det_gen);
            y_person1 += 0 + detNoise(det_gen);
            break;
    }

    // Person 2
    switch(switch_ID2){
        case 0: // Wait for rviz to start up
            x_person2 =  0   + detNoise(det_gen);
            y_person2 = -1.5 + detNoise(det_gen);
            delay2 += dt;
            if(delay2 > 4){
                delay2 = 0;
                switch_ID2 = 1;
            }
            break;
        case 1: // Move in positive x-direction
            x_person2 += dt * vel_person + detNoise(det_gen);
            y_person2 += 0               + detNoise(det_gen);
            if(x_person2 > 1.5){
                switch_ID2 = 2;
            }
            break;
        case 2: // Turn
            x_person2 +=  std::cos(delay2 * 0.5 * M_PI) *  dt * vel_person + detNoise(det_gen);
            y_person2 += -std::sin(delay2 * 0.5 * M_PI) * -dt * vel_person + detNoise(det_gen);
            delay2 += dt;
            if(delay2 > 1){
                delay2 = 0;
                switch_ID2 = 3;
            }
            break;
        case 3: // Move in positive y-direction
            x_person2 += 0;
            y_person2 += dt * vel_person;
            if(y_person2 > 1){
                switch_ID2 = 4;
            }
            break;
        case 4: // Turn
            x_person2 += -std::sin(delay2 * 0.5 * M_PI) *  dt * vel_person + detNoise(det_gen);
            y_person2 += -std::cos(delay2 * 0.5 * M_PI) * -dt * vel_person + detNoise(det_gen);
            delay2 += dt;
            if(delay2 > 1){
                delay2 = 0;
                switch_ID2 = 5;
            }
            break;
        case 5: // Move in negative x-direction
            x_person2 += -dt * vel_person + detNoise(det_gen);
            y_person2 += 0                + detNoise(det_gen);
            if(x_person2 < 0){
                switch_ID2 = 6;
            }
            break;
        case 6: // End position
            x_person2 += 0 + detNoise(det_gen);
            y_person2 += 0 + detNoise(det_gen);
            break;
    }

    // Add evidence person
    if(y_person1 < -0.55 || y_person1 > 0.55) {
        addEvidence(world_evidence, x_person1, y_person1, 1, "person", "red");
    }
    if(y_person2 < -0.55 || y_person2 > 0.55) {
        addEvidence(world_evidence, x_person2, y_person2, 1, "person", "red");
    }

    // Send info about OOS area
    OOS_range_.data.clear();
    OOS_range_.data.push_back( 1.6); // OOS_min_x
    OOS_range_.data.push_back( 3.0); // OOS_max_x
    OOS_range_.data.push_back(-0.6); // OOS_min_y
    OOS_range_.data.push_back( 0.6); // OOS_max_y
    OOS_publisher_.publish(OOS_range_);

    return;
}

/// SITUATION 5: Two persons are occluded at the same time, sensor to distinguish
void Trajectories::Situation5(wire_msgs::WorldEvidence& world_evidence){
    // Define velocity person
    double vel_person = 0.5;

    // Random sets
    std::normal_distribution<> detNoise{0,noiseSTD}    ; // Gaussian noise mean = 0, std = noiseSTD

    // Person 1
    switch(switch_ID1){
        case 0: // Wait for rviz to start up
            x_person1 = 0   + detNoise(det_gen);
            y_person1 = 1.5 + detNoise(det_gen);
            delay1 += dt;
            if(delay1 > 4){
                delay1 = 0;
                switch_ID1 = 1;
            }
            break;
        case 1: // Move in positive x-direction
            x_person1 += dt * vel_person + detNoise(det_gen);
            y_person1 += 0               + detNoise(det_gen);
            if(x_person1 > 1.5){
                switch_ID1 = 2;
            }
            break;
        case 2: // Turn
            x_person1 += std::cos(delay1 * 0.5 * M_PI) *  dt * vel_person + detNoise(det_gen);
            y_person1 += std::sin(delay1 * 0.5 * M_PI) * -dt * vel_person + detNoise(det_gen);
            delay1 += dt;
            if(delay1 > 1){
                delay1 = 0;
                switch_ID1 = 3;
            }
            break;
        case 3: // Move in negative y-direction
            x_person1 += 0;
            y_person1 += -dt * vel_person;
            if(y_person1 < -1){
                switch_ID1 = 4;
            }
            break;
        case 4: // Turn
            x_person1 += -std::sin(delay1 * 0.5 * M_PI) *  dt * vel_person + detNoise(det_gen);
            y_person1 +=  std::cos(delay1 * 0.5 * M_PI) * -dt * vel_person + detNoise(det_gen);
            delay1 += dt;
            if(delay1 > 1){
                delay1 = 0;
                switch_ID1 = 5;
            }
            break;
        case 5: // Move in negative x-direction
            x_person1 += -dt * vel_person + detNoise(det_gen);
            y_person1 += 0                + detNoise(det_gen);
            if(x_person1 < 0){
                switch_ID1 = 6;
            }
            break;
        case 6: // End position
            x_person1 += 0 + detNoise(det_gen);
            y_person1 += 0 + detNoise(det_gen);
            break;
    }

    // Person 2
    switch(switch_ID2){
        case 0: // Wait for rviz to start up
            x_person2 =  0   + detNoise(det_gen);
            y_person2 = -1.5 + detNoise(det_gen);
            delay2 += dt;
            if(delay2 > 4){
                delay2 = 0;
                switch_ID2 = 1;
            }
            break;
        case 1: // Move in positive x-direction
            x_person2 += dt * vel_person + detNoise(det_gen);
            y_person2 += 0               + detNoise(det_gen);
            if(x_person2 > 1.5){
                switch_ID2 = 2;
            }
            break;
        case 2: // Turn
            x_person2 +=  std::cos(delay2 * 0.5 * M_PI) *  dt * vel_person + detNoise(det_gen);
            y_person2 += -std::sin(delay2 * 0.5 * M_PI) * -dt * vel_person + detNoise(det_gen);
            delay2 += dt;
            if(delay2 > 1){
                delay2 = 0;
                switch_ID2 = 3;
            }
            break;
        case 3: // Move in positive y-direction
            x_person2 += 0;
            y_person2 += dt * vel_person;
            if(y_person2 > 1){
                switch_ID2 = 4;
            }
            break;
        case 4: // Turn
            x_person2 += -std::sin(delay2 * 0.5 * M_PI) *  dt * vel_person + detNoise(det_gen);
            y_person2 += -std::cos(delay2 * 0.5 * M_PI) * -dt * vel_person + detNoise(det_gen);
            delay2 += dt;
            if(delay2 > 1){
                delay2 = 0;
                switch_ID2 = 5;
            }
            break;
        case 5: // Move in negative x-direction
            x_person2 += -dt * vel_person + detNoise(det_gen);
            y_person2 += 0                + detNoise(det_gen);
            if(x_person2 < 0){
                switch_ID2 = 6;
            }
            break;
        case 6: // End position
            x_person2 += 0 + detNoise(det_gen);
            y_person2 += 0 + detNoise(det_gen);
            break;
    }

    // Add evidence person
    if(y_person1 < -0.55 || y_person2 < -0.55) {
        addEvidence(world_evidence, x_person1, y_person1, 1, "person", "red");
    }
    if(y_person1 >  0.55 || y_person2 >  0.55) {
        addEvidence(world_evidence, x_person2, y_person2, 1, "person", "green");
    }

    // Send info about OOS area
    OOS_range_.data.clear();
    OOS_range_.data.push_back( 1.6); // OOS_min_x
    OOS_range_.data.push_back( 3.0); // OOS_max_x
    OOS_range_.data.push_back(-0.6); // OOS_min_y
    OOS_range_.data.push_back( 0.6); // OOS_max_y
    OOS_publisher_.publish(OOS_range_);

    return;
}

/// SITUATION 6: Two persons occluding each other
void Trajectories::Situation6(wire_msgs::WorldEvidence& world_evidence){
    // Define velocity person
    double vel_person = 0.5;

    // Random sets
//    std::normal_distribution<> detNoise{0,noiseSTD}    ; // Gaussian noise mean = 0, std = noiseSTD
    std::normal_distribution<> detNoise{0,0}    ; // Gaussian noise mean = 0, std = 0

    // Person 1
    switch(switch_ID1){
        case 0: // Wait for rviz to start up
            x_person1 = 0   + detNoise(det_gen);
            y_person1 = 1.5 + detNoise(det_gen);
            delay1 += dt;
            if(delay1 > 4){
                delay1 = 0;
                switch_ID1 = 1;
            }
            break;
        case 1: // Move in positive x-direction
            x_person1 += dt * vel_person + detNoise(det_gen);
            y_person1 += 0               + detNoise(det_gen);
            if(x_person1 > 1.0){
                switch_ID1 = 2;
            }
            break;
        case 2: // Turn
            x_person1 += std::cos(delay1 * 0.5 * M_PI) *  dt * vel_person + detNoise(det_gen);
            y_person1 += std::sin(delay1 * 0.5 * M_PI) * -dt * vel_person + detNoise(det_gen);
            delay1 += dt;
            if(delay1 > 1){
                delay1 = 0;
                switch_ID1 = 3;
            }
            break;
        case 3: // Move in negative y-direction
            x_person1 += 0;
            y_person1 += -dt * vel_person;
            if(y_person1 < -1){
                switch_ID1 = 4;
            }
            break;
        case 4: // Turn
            x_person1 += -std::sin(delay1 * 0.5 * M_PI) *  dt * vel_person + detNoise(det_gen);
            y_person1 +=  std::cos(delay1 * 0.5 * M_PI) * -dt * vel_person + detNoise(det_gen);
            delay1 += dt;
            if(delay1 > 1){
                delay1 = 0;
                switch_ID1 = 5;
            }
            break;
        case 5: // Move in negative x-direction
            x_person1 += -dt * vel_person + detNoise(det_gen);
            y_person1 += 0                + detNoise(det_gen);
            if(x_person1 < 0){
                switch_ID1 = 6;
            }
            break;
        case 6: // End position
            x_person1 += 0 + detNoise(det_gen);
            y_person1 += 0 + detNoise(det_gen);
            break;
    }

    // Person 2
    switch(switch_ID2){
        case 0: // Wait for rviz to start up
            x_person2 = 2    + detNoise(det_gen);
            y_person2 = 2.75 + detNoise(det_gen);
            delay2 += dt;
            if(delay2 > 4){
                delay2 = 0;
                switch_ID2 = 1;
            }
            break;
        case 1: // Move in negative y-direction
            x_person2 += 0;
            y_person2 += -dt * vel_person;
            if(y_person2 < -2.75){
                switch_ID2 = 2;
            }
            break;
        case 2: // End position
            x_person2 += 0 + detNoise(det_gen);
            y_person2 += 0 + detNoise(det_gen);
            break;
    }

    // Add evidence person
    addEvidence(world_evidence, x_person1, y_person1, 1, "person", "red");
    if(y_person2 < y_person1 - 0.2 || y_person2 > y_person1 + 0.2) {
        addEvidence(world_evidence, x_person2, y_person2, 1, "person", "green");
    }

    // Send info about OOS area
    OOS_range_.data.clear();
    // TODO: fix that distribution of occluded object cannot be associated with object that occludes
    OOS_range_.data.push_back(x_person1 + 0.1)  ; // OOS_min_x
    OOS_range_.data.push_back(3.0)              ; // OOS_max_x
    OOS_range_.data.push_back(y_person1 - 0.25)  ; // OOS_min_y
    OOS_range_.data.push_back(y_person1 + 0.25)  ; // OOS_max_y
    OOS_publisher_.publish(OOS_range_);

    return;
}

/// SITUATION 7: Three persons occluding each other
void Trajectories::Situation7(wire_msgs::WorldEvidence& world_evidence){
    // Define velocity person
    double vel_person = 0.5;

    // Random sets
//    std::normal_distribution<> detNoise{0,noiseSTD}    ; // Gaussian noise mean = 0, std = noiseSTD
    std::normal_distribution<> detNoise{0,0}    ; // Gaussian noise mean = 0, std = 0

    // Person 1
    switch(switch_ID1){
        case 0: // Wait for rviz to start up
            x_person1 = 0   + detNoise(det_gen);
            y_person1 = 1.5 + detNoise(det_gen);
            delay1 += dt;
            if(delay1 > 4){
                delay1 = 0;
                switch_ID1 = 1;
            }
            break;
        case 1: // Move in positive x-direction
            x_person1 += dt * vel_person + detNoise(det_gen);
            y_person1 += 0               + detNoise(det_gen);
            if(x_person1 > 1.0){
                switch_ID1 = 2;
            }
            break;
        case 2: // Turn
            x_person1 += std::cos(delay1 * 0.5 * M_PI) *  dt * vel_person + detNoise(det_gen);
            y_person1 += std::sin(delay1 * 0.5 * M_PI) * -dt * vel_person + detNoise(det_gen);
            delay1 += dt;
            if(delay1 > 1){
                delay1 = 0;
                switch_ID1 = 3;
            }
            break;
        case 3: // Move in negative y-direction
            x_person1 += 0;
            y_person1 += -dt * vel_person;
            if(y_person1 < -1){
                switch_ID1 = 4;
            }
            break;
        case 4: // Turn
            x_person1 += -std::sin(delay1 * 0.5 * M_PI) *  dt * vel_person + detNoise(det_gen);
            y_person1 +=  std::cos(delay1 * 0.5 * M_PI) * -dt * vel_person + detNoise(det_gen);
            delay1 += dt;
            if(delay1 > 1){
                delay1 = 0;
                switch_ID1 = 5;
            }
            break;
        case 5: // Move in negative x-direction
            x_person1 += -dt * vel_person + detNoise(det_gen);
            y_person1 += 0                + detNoise(det_gen);
            if(x_person1 < 0){
                switch_ID1 = 6;
            }
            break;
        case 6: // End position
            x_person1 += 0 + detNoise(det_gen);
            y_person1 += 0 + detNoise(det_gen);
            break;
    }

    // Person 2
    switch(switch_ID2){
        case 0: // Wait for rviz to start up
            x_person2 = 2    + detNoise(det_gen);
            y_person2 = 2.75 + detNoise(det_gen);
            delay2 += dt;
            if(delay2 > 4){
                delay2 = 0;
                switch_ID2 = 1;
            }
            break;
        case 1: // Move in negative y-direction
            x_person2 += 0;
            y_person2 += -dt * vel_person;
            if(y_person2 < -2.75){
                switch_ID2 = 2;
            }
            break;
        case 2: // End position
            x_person2 += 0 + detNoise(det_gen);
            y_person2 += 0 + detNoise(det_gen);
            break;
    }

    // Person 3
    switch(switch_ID3){
        case 0: // Wait for rviz to start up
            x_person3 = 2.5 + detNoise(det_gen);
            y_person3 = 0.0 + detNoise(det_gen);
            if(y_person1 < -0.15){
                switch_ID3 = 1;
            }
            break;
        case 1: // Move in negative y-direction
            x_person3 += -0.7 * dt * vel_person;
            y_person3 += -1.0 * dt * vel_person;
            if(y_person3 < -2.3){
                switch_ID3 = 2;
            }
            break;
        case 2: // End position
            x_person3 += 0 + detNoise(det_gen);
            y_person3 += 0 + detNoise(det_gen);
            break;
    }

    // Add evidence person
    addEvidence(world_evidence, x_person1, y_person1, 1, "person", "red");
    if(y_person2 < y_person1 - 0.2 || y_person2 > y_person1 + 0.2) {
        addEvidence(world_evidence, x_person2, y_person2, 1, "person", "green");
    }
    if(y_person3 < y_person1 - 0.2 || y_person3 > y_person1 + 0.2) {
        addEvidence(world_evidence, x_person3, y_person3, 1, "person", "blue");
    }

    // Send info about OOS area
    OOS_range_.data.clear();
    // TODO: fix that distribution of occluded object cannot be associated with object that occludes
    OOS_range_.data.push_back(x_person1 + 0.1)  ; // OOS_min_x
    OOS_range_.data.push_back(3.0)              ; // OOS_max_x
    OOS_range_.data.push_back(y_person1 - 0.25)  ; // OOS_min_y
    OOS_range_.data.push_back(y_person1 + 0.25)  ; // OOS_max_y
    OOS_publisher_.publish(OOS_range_);

    return;
}