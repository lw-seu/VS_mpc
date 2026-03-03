#include <memory>

#include "px4_interface.h"
#include "pipeline_inspection.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "inspection_node");

    std::shared_ptr<OffboardMode> offboardMode = 
        std::make_shared<OffboardMode>(new PipelineInspection());

    return offboardMode->mainLoop();
}