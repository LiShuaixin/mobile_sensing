#include <system.h>

int main (int argc, char ** argv)
{
    Mobile_Sensing::System mobile_sensing(argc, argv, Mobile_Sensing::HDL_64);
    
    mobile_sensing.run();

    return (0);
}