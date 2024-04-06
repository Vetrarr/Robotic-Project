#include <datmo.h>

//UPDATE
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void datmo::update() 
{

    // we wait for new data of the laser and of the robot_moving_node to perform laser processing
    if ( new_laser && new_robot ) 
    {
        if (!static_background_stored) {
            store_background();
            static_background_stored = true;
        }

        ROS_INFO("\n");
        ROS_INFO("New data of laser received");
        ROS_INFO("New data of robot_moving received");        

        detect_motion();
        display_motion();

        perform_clustering();
        display_clustering();

        detect_legs();
        display_legs();

        detect_persons(); 
        display_persons();

        if(is_person_tracked){
            track_a_person(); // process all the persons, even static
            display_a_tracked_person();

            ROS_INFO("===================TRACKING A PERSON=====================");     

            if (frequency <= frequency_init || uncertainty >= uncertainty_max) { // lost the person
                is_person_tracked = false;

                geometry_msgs::Point person_lost;
                person_lost.x = -100;
                person_lost.y = -100;

                pub_datmo.publish(person_lost);
            }
        } else if (!is_person_tracked || (!current_robot_moving && previous_robot_moving)) {
            detect_a_moving_person();

            frequency = frequency_init;
            uncertainty = uncertainty_min;
        }



        // if the robot is not moving then we can perform moving person detection
        if (!current_robot_moving)
        {
            ROS_INFO("robot is not moving");

            // if the robot is not moving then we can perform moving person detection
            // DO NOT FORGET to store the background but when ???
            if (previous_robot_moving)
            {
                store_background();
                ROS_INFO("robot was moving");
            }
            else
            {
                ROS_INFO("robot was not moving");
            }
        }
        else
        {
            ROS_INFO("robot is moving");

            // IMPOSSIBLE TO DETECT MOTIONS because the base is moving
            // what is the value of dynamic table for each hit of the laser ?
            if (!previous_robot_moving)
            {
                ROS_INFO("robot was not moving");
            }
            else
            {
                ROS_INFO("robot was moving");
            }
        }
        
        
// at first detection, if found - make tracking in place

        reset_motion();

        populateMarkerReference();
        new_laser = false;
        new_robot = false;
        previous_robot_moving = current_robot_moving;       
        
    }
    else
    {
        if ( !init_laser )
            ROS_WARN("waiting for laser data: run a rosbag");
        else
            if ( !init_robot )
                ROS_WARN("waiting for robot_moving_node: rosrun follow_me robot_moving_node");
    }

}// update

int main(int argc, char **argv)
{

    ros::init(argc, argv, "detection_node");

    ROS_INFO("waiting for detection of a moving person");
    datmo bsObject;

    ros::spin();

    return 0;
}
