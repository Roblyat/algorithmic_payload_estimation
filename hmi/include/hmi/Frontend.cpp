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

    if (ImGui::BeginTabBar("Tabs", ImGuiTabBarFlags_FittingPolicyScroll))
    {
        if (ImGui::BeginTabItem("Control Panel"))
        {
            currentTab = 0;
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Measurement Panel"))
        {
            currentTab = 1;
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Plot Panel"))
        {
            currentTab = 2;
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }

    // Ensure the content is placed right below the tabs
    ImGui::Separator();

    controlPanel.render();

    // Render the currently selected tab
    // switch (currentTab)
    // {
    // case 0:
    //     controlPanel.render();
    //     break;
    // case 1:
    //     measurementPanel.render();
    //     break;
    // case 2:
    //     plotPanel.render();
    //     break;
    // }

    ImGui::End();
}