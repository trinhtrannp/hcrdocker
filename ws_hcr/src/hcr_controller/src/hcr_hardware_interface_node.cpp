#include <hcr_hardware_interface/hcr_hardware_interface.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "hcr_hardware_interface");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    hcr_hardware_interface::HCRHardwareInterface HCR_HWInterface(nh);
    ros::spin();
    return 0;
}
