/********************************************************
 *                                                      *
 *                                                      *
 *      state_machine_destination.h           		*
 *                                                      *
 *              Jesus Savage                            *
 *              Diego Cordero                           *
 *              FI-UNAM                                 *
 *              13-2-2019                               *
 *                                                      *
 ********************************************************/

#define THRESHOLD_DEST 7.1

bool sm_destination(float max_intensity, int dest, movement *movements, int *next_state, float Mag_Advance, float max_twist)
{
    int state = *next_state;
    bool finished = false;
    std::cout << "State.->" << state << std::endl;
    switch ( state ) {
        case 1:
            if (max_intensity > THRESHOLD_DEST) {
                *movements=generate_output(STOP, Mag_Advance, max_twist);
                printf("\n **************** Reached light source ******************************\n");
                *next_state = 1;
                finished = true;
            } else {
                *movements=generate_output(FORWARD, Mag_Advance, max_twist);
                *next_state = 2;
            }
            break;
	    case 2:
            if (dest == GO_RIGHT){
                *movements=generate_output(RIGHT, Mag_Advance,max_twist);
                *next_state = 3;
            } else if (dest == GO_LEFT){
                *movements=generate_output(LEFT, Mag_Advance,max_twist);
                *next_state = 4;
            } else if (dest == GO_FOWARD_RIGHT){
                *movements=generate_output(FORWARD, Mag_Advance,max_twist);
                *next_state = 3;
            } else if (dest == GO_FOWARD_LEFT){
                *movements=generate_output(FORWARD, Mag_Advance,max_twist);
                *next_state = 4;
            } else if (dest == GO_FOWARD){
                *movements=generate_output(FORWARD, Mag_Advance,max_twist);
                *next_state = 1;
            }
            break;
        case 3:
            *movements=generate_output(RIGHT, Mag_Advance, max_twist);
            *next_state = 1;
            break;
        case 4:
            *movements=generate_output(LEFT, Mag_Advance,max_twist);
            *next_state = 1;
            break;

        default:
            *movements=generate_output(STOP, Mag_Advance,max_twist);
            *next_state = 1;
            break;
    }

    return finished;
}