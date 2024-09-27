#include "Frontend.h"

Frontend::Frontend(ros::NodeHandle &nh)
     : controlPanel(nh), currentTab(0)
{

}

void Frontend::render()
{
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize);
    ImGui::Begin("Main Window", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar);

    // Ensure the content is placed right below the tabs
    ImGui::Separator();

    controlPanel.render();

    ImGui::End();
}