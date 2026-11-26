#include "DesktopSessionController.hpp"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl2.h"

#ifdef _WIN32
#include <windows.h>
#endif
#include <GLFW/glfw3.h>
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

namespace {

constexpr float kPi = 3.14159265358979323846f;

struct OrbitCamera {
    float distance = 14.0f;
    float yaw_rad = -0.78f;
    float pitch_rad = 0.55f;
    float target[3] = {0.0f, 0.0f, 0.0f};
};

void jetColor(float t, float& r, float& g, float& b) {
    float x = std::clamp(t, 0.0f, 1.0f);
    r = std::clamp(1.5f - std::abs(4.0f * x - 3.0f), 0.0f, 1.0f);
    g = std::clamp(1.5f - std::abs(4.0f * x - 2.0f), 0.0f, 1.0f);
    b = std::clamp(1.5f - std::abs(4.0f * x - 1.0f), 0.0f, 1.0f);
}

void setPerspectiveGL(double fovYDeg, double aspect, double zNear, double zFar) {
    double fH = std::tan(fovYDeg / 360.0 * kPi) * zNear;
    double fW = fH * aspect;
    glFrustum(-fW, fW, -fH, fH, zNear, zFar);
}

void setLookAtGL(float ex, float ey, float ez, float tx, float ty, float tz) {
    float fx = tx - ex, fy = ty - ey, fz = tz - ez;
    float fl = std::sqrt(fx * fx + fy * fy + fz * fz);
    if (fl < 1e-6f) return;
    fx /= fl; fy /= fl; fz /= fl;

    float ux = 0.0f, uy = 0.0f, uz = 1.0f;
    float sx = fy * uz - fz * uy;
    float sy = fz * ux - fx * uz;
    float sz = fx * uy - fy * ux;
    float sl = std::sqrt(sx * sx + sy * sy + sz * sz);
    if (sl < 1e-6f) { sx = 1.0f; sy = 0.0f; sz = 0.0f; sl = 1.0f; }
    sx /= sl; sy /= sl; sz /= sl;

    float vx = sy * fz - sz * fy;
    float vy = sz * fx - sx * fz;
    float vz = sx * fy - sy * fx;

    float m[16] = {
        sx, vx, -fx, 0.0f,
        sy, vy, -fy, 0.0f,
        sz, vz, -fz, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    };
    glMultMatrixf(m);
    glTranslatef(-ex, -ey, -ez);
}

void applyAppleGlassImGuiStyle(int glass_opacity_pct) {
    ImGuiStyle& s = ImGui::GetStyle();
    s.WindowRounding = 10.0f;
    s.FrameRounding = 7.0f;
    s.PopupRounding = 8.0f;
    s.WindowBorderSize = 1.2f;
    s.FrameBorderSize = 1.0f;
    s.WindowPadding = ImVec2(12, 10);
    s.ItemSpacing = ImVec2(8, 6);

    float bg_alpha = (glass_opacity_pct == 0) ? 0.12f : (0.28f + (glass_opacity_pct / 100.0f) * 0.65f);
    ImVec4 gold(186.0f / 255.0f, 161.0f / 255.0f, 100.0f / 255.0f, 1.0f);
    ImVec4 goldBright(229.0f / 255.0f, 201.0f / 255.0f, 131.0f / 255.0f, 1.0f);
    ImVec4 darkBg(16.0f / 255.0f, 16.0f / 255.0f, 22.0f / 255.0f, bg_alpha);

    s.Colors[ImGuiCol_WindowBg] = darkBg;
    s.Colors[ImGuiCol_PopupBg] = ImVec4(16.0f / 255.0f, 16.0f / 255.0f, 22.0f / 255.0f, 0.94f);
    s.Colors[ImGuiCol_Border] = ImVec4(gold.x, gold.y, gold.z, 0.65f);
    s.Colors[ImGuiCol_TitleBg] = ImVec4(18.0f / 255.0f, 18.0f / 255.0f, 24.0f / 255.0f, 0.92f);
    s.Colors[ImGuiCol_TitleBgActive] = ImVec4(28.0f / 255.0f, 26.0f / 255.0f, 34.0f / 255.0f, 0.95f);
    s.Colors[ImGuiCol_Text] = ImVec4(252.0f / 255.0f, 251.0f / 255.0f, 248.0f / 255.0f, 1.0f);
    s.Colors[ImGuiCol_Button] = ImVec4(gold.x, gold.y, gold.z, 0.30f);
    s.Colors[ImGuiCol_ButtonHovered] = ImVec4(goldBright.x, goldBright.y, goldBright.z, 0.55f);
    s.Colors[ImGuiCol_ButtonActive] = ImVec4(gold.x, gold.y, gold.z, 0.80f);
    s.Colors[ImGuiCol_Header] = ImVec4(gold.x, gold.y, gold.z, 0.35f);
    s.Colors[ImGuiCol_HeaderHovered] = ImVec4(goldBright.x, goldBright.y, goldBright.z, 0.50f);
    s.Colors[ImGuiCol_FrameBg] = ImVec4(1.0f, 1.0f, 1.0f, 0.08f);
}

void drawGroundGrid() {
    glColor4f(0.28f, 0.30f, 0.36f, 0.45f);
    glBegin(GL_LINES);
    for (int i = -15; i <= 15; ++i) {
        glVertex3f(static_cast<float>(i), -15.0f, 0.0f);
        glVertex3f(static_cast<float>(i),  15.0f, 0.0f);
        glVertex3f(-15.0f, static_cast<float>(i), 0.0f);
        glVertex3f( 15.0f, static_cast<float>(i), 0.0f);
    }
    glEnd();

    // World RGB axes
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    glColor3f(0.95f, 0.30f, 0.30f); glVertex3f(0, 0, 0); glVertex3f(1.5f, 0, 0);
    glColor3f(0.30f, 0.95f, 0.45f); glVertex3f(0, 0, 0); glVertex3f(0, 1.5f, 0);
    glColor3f(0.30f, 0.70f, 0.98f); glVertex3f(0, 0, 0); glVertex3f(0, 0, 1.5f);
    glEnd();
    glLineWidth(1.0f);
}

}  // namespace

int main(int argc, char** argv) {
    bool headless_smoke_test = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--smoke-test") == 0) headless_smoke_test = true;
    }

    avscan::desktop::DesktopSessionController controller;
    if (headless_smoke_test) {
        auto diag = controller.runAiryDiagnostics();
        auto profiles = avscan::core::DeviceProfileRegistry::instance().allProfiles();
        std::printf("[PASS] AV-SLAM Desktop Smoke Test OK | ActiveRig=%s | Profiles=%zu | Project=%s/%s | Adapter=%s\n",
                    controller.activeDeviceProfileDisplayName().c_str(),
                    profiles.size(),
                    controller.activeProjectName().c_str(),
                    controller.activeScanName().c_str(),
                    diag.adapter_detected ? "ONLINE" : "OFFLINE");
        return 0;
    }

    if (!glfwInit()) {
        std::fprintf(stderr, "Failed to initialize GLFW.\n");
        return 1;
    }

    GLFWwindow* window = glfwCreateWindow(1440, 900, "AV SLAM — Proprietary Multi-Sensor Desktop (Windows / Linux)", nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL2_Init();

    int glass_opacity_pct = 25;
    applyAppleGlassImGuiStyle(glass_opacity_pct);

    OrbitCamera cam;
    int color_mode = 0; // 0=RGB, 1=Reflectivity, 2=Elevation Z
    float point_size = 3.0f;

    bool show_projects_modal = false;
    bool show_cslam_window = false;
    bool show_airy_window = false;
    bool show_calib_window = false;
    bool show_export_modal = false;

    char new_proj_name[128] = "Site_Project_01";
    char new_proj_site[128] = "Building A";
    char new_scan_name[128] = "Scan_02";
    char new_scan_floor[64] = "Level 01";
    char peer_ip_buf[64] = "192.168.1.50";
    char airy_ip_buf[64] = "192.168.1.200";
    char host_ip_buf[64] = "192.168.1.102";
    std::string export_status;
    avscan::sensors::AiryDiscoveryReport last_diag{};

    double last_mouse_x = 0.0, last_mouse_y = 0.0;
    bool dragging_orbit = false, dragging_pan = false;

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // Viewport mouse controls (when ImGui is not capturing mouse)
        ImGuiIO& io = ImGui::GetIO();
        if (!io.WantCaptureMouse) {
            double mx, my;
            glfwGetCursorPos(window, &mx, &my);
            if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
                if (dragging_orbit) {
                    cam.yaw_rad -= static_cast<float>(mx - last_mouse_x) * 0.006f;
                    cam.pitch_rad = std::clamp(cam.pitch_rad + static_cast<float>(my - last_mouse_y) * 0.006f, -1.50f, 1.50f);
                }
                dragging_orbit = true;
            } else {
                dragging_orbit = false;
            }
            if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS ||
                glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS) {
                if (dragging_pan) {
                    float scale = cam.distance * 0.002f;
                    float dx = static_cast<float>(mx - last_mouse_x);
                    float dy = static_cast<float>(my - last_mouse_y);
                    cam.target[0] -= dx * scale * std::sin(cam.yaw_rad);
                    cam.target[1] += dx * scale * std::cos(cam.yaw_rad);
                    cam.target[2] += dy * scale;
                }
                dragging_pan = true;
            } else {
                dragging_pan = false;
            }
            if (std::abs(io.MouseWheel) > 0.01f) {
                cam.distance = std::clamp(cam.distance * (io.MouseWheel > 0 ? 0.88f : 1.14f), 1.0f, 180.0f);
            }
            last_mouse_x = mx;
            last_mouse_y = my;
        } else {
            dragging_orbit = dragging_pan = false;
        }

        // Snapshot latest cloud & trajectory
        auto cloud = controller.latestGlobalPointCloud();
        auto traj = controller.currentTrajectory();
        auto pkt = controller.currentPacketStats();
        auto qual = controller.latestQualityReport();
        auto slam_stats = controller.currentAvSlamStats();
        auto cslam_stats = controller.cslamStats();

        // Render 3D OpenGL Viewport
        int fb_w = 1280, fb_h = 720;
        glfwGetFramebufferSize(window, &fb_w, &fb_h);
        glViewport(0, 0, fb_w, fb_h);
        glClearColor(0.05f, 0.06f, 0.09f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        double aspect = (fb_h > 0) ? (static_cast<double>(fb_w) / static_cast<double>(fb_h)) : 1.6;
        setPerspectiveGL(60.0, aspect, 0.1, 300.0);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        float ex = cam.target[0] + cam.distance * std::cos(cam.pitch_rad) * std::cos(cam.yaw_rad);
        float ey = cam.target[1] + cam.distance * std::cos(cam.pitch_rad) * std::sin(cam.yaw_rad);
        float ez = cam.target[2] + cam.distance * std::sin(cam.pitch_rad);
        setLookAtGL(ex, ey, ez, cam.target[0], cam.target[1], cam.target[2]);

        drawGroundGrid();

        if (!cloud.empty()) {
            float min_z = 1e6f, max_z = -1e6f;
            size_t stride = std::max<size_t>(1, cloud.size() / 400000);
            for (size_t i = 0; i < cloud.size(); i += stride) {
                min_z = std::min(min_z, static_cast<float>(cloud[i].z));
                max_z = std::max(max_z, static_cast<float>(cloud[i].z));
            }
            float z_span = std::max(0.5f, max_z - min_z);

            glPointSize(point_size);
            glBegin(GL_POINTS);
            for (size_t i = 0; i < cloud.size(); i += stride) {
                const auto& p = cloud[i];
                if (color_mode == 1) {
                    float r, g, b;
                    jetColor(p.intensity / 255.0f, r, g, b);
                    glColor3f(r, g, b);
                } else if (color_mode == 2) {
                    float r, g, b;
                    jetColor((static_cast<float>(p.z) - min_z) / z_span, r, g, b);
                    glColor3f(r, g, b);
                } else {
                    glColor3f(p.r / 255.0f, p.g / 255.0f, p.b / 255.0f);
                }
                glVertex3f(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z));
            }
            glEnd();
        }

        if (traj.size() >= 2) {
            glLineWidth(2.5f);
            glColor3f(0.22f, 0.74f, 0.97f);
            glBegin(GL_LINE_STRIP);
            for (const auto& st : traj) {
                glVertex3f(static_cast<float>(st.T_world_body.t.x),
                           static_cast<float>(st.T_world_body.t.y),
                           static_cast<float>(st.T_world_body.t.z));
            }
            glEnd();
            glLineWidth(1.0f);
        }

        // Start ImGui HUD Frame
        ImGui_ImplOpenGL2_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Top Navigation Bar
        if (ImGui::BeginMainMenuBar()) {
            ImGui::TextColored(ImVec4(0.90f, 0.79f, 0.51f, 1.0f), "AV SLAM DESKTOP");
            ImGui::Separator();
            if (ImGui::Button("Projects")) show_projects_modal = !show_projects_modal;
            char cslam_btn[48];
            if (controller.isCslamEnabled()) {
                std::snprintf(cslam_btn, sizeof(cslam_btn), "C-SLAM (%zu)", cslam_stats.discovered_peers);
            } else {
                std::snprintf(cslam_btn, sizeof(cslam_btn), "Connect (C-SLAM)");
            }
            if (ImGui::Button(cslam_btn)) show_cslam_window = !show_cslam_window;
            if (ImGui::Button("Airy Net")) {
                show_airy_window = !show_airy_window;
                if (show_airy_window) last_diag = controller.runAiryDiagnostics();
            }
            if (ImGui::Button("Calibrate")) show_calib_window = !show_calib_window;

            if (ImGui::BeginMenu("Rig Profile")) {
                const auto& profiles = avscan::core::DeviceProfileRegistry::instance().allProfiles();
                std::string cur_id = controller.activeDeviceProfileId();
                for (const auto& p : profiles) {
                    bool sel = (p.profile_id == cur_id);
                    if (ImGui::MenuItem(p.display_name.c_str(), nullptr, sel)) {
                        controller.setActiveDeviceProfile(p.profile_id);
                    }
                }
                ImGui::EndMenu();
            }

            char glass_lbl[32];
            std::snprintf(glass_lbl, sizeof(glass_lbl), "Glass %d%%", glass_opacity_pct);
            if (ImGui::Button(glass_lbl)) {
                if (glass_opacity_pct == 0) glass_opacity_pct = 12;
                else if (glass_opacity_pct == 12) glass_opacity_pct = 25;
                else if (glass_opacity_pct == 25) glass_opacity_pct = 50;
                else if (glass_opacity_pct == 50) glass_opacity_pct = 80;
                else glass_opacity_pct = 0;
                applyAppleGlassImGuiStyle(glass_opacity_pct);
            }
            ImGui::EndMainMenuBar();
        }

        // Top-Left HUD Status & Metrics Card
        ImGui::SetNextWindowPos(ImVec2(14, 34), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(430, 150), ImGuiCond_Always);
        ImGui::Begin("##HudStatus", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);
        ImGui::TextColored(ImVec4(0.90f, 0.79f, 0.51f, 1.0f), "%s  |  %s (%s)",
                           controller.activeProjectName().c_str(),
                           controller.activeScanName().c_str(),
                           controller.activeScanFloorLevel().c_str());
        ImGui::Text("RIG: %s", controller.activeDeviceProfileDisplayName().c_str());
        ImGui::Text("PTS: %zu  |  LOOPS: %zu  |  LOSS: %.2f%%  |  %.1f ms/sweep",
                    cloud.size(), controller.loopClosureCount(),
                    pkt.msopPacketLossRatio() * 100.0, slam_stats.avg_frame_ms);
        ImGui::Text("CAM: %s (%.1f FPS)", controller.cameraStatusSummary().c_str(), controller.cameraMeasuredFps());
        ImGui::TextColored(ImVec4(0.29f, 0.87f, 0.50f, 1.0f), "QUALITY: %.0f%%  |  RMSE: %.3f m  |  DRIFT RISK: %.2f m",
                           qual.overall_quality_score * 100.0, qual.registration_rmse_m, qual.estimated_drift_risk_m);
        ImGui::End();

        // Top-Right 2D Floor Plan Radar Widget
        auto plan = controller.latestFloorPlan();
        ImGui::SetNextWindowPos(ImVec2(static_cast<float>(fb_w) - 244.0f, 34.0f), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(230, 230), ImGuiCond_Always);
        ImGui::Begin("2D Floor Plan Radar", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);
        ImDrawList* dl = ImGui::GetWindowDrawList();
        ImVec2 wp = ImGui::GetCursorScreenPos();
        ImVec2 ws = ImGui::GetContentRegionAvail();
        if (plan.width_cells > 0 && plan.height_cells > 0 && !plan.occupancy_cells.empty()) {
            float sx = ws.x / static_cast<float>(plan.width_cells);
            float sy = ws.y / static_cast<float>(plan.height_cells);
            float cs = std::min(sx, sy);
            for (int y = 0; y < plan.height_cells; ++y) {
                for (int x = 0; x < plan.width_cells; ++x) {
                    uint8_t v = plan.occupancy_cells[static_cast<size_t>(y * plan.width_cells + x)];
                    if (v < 2) continue;
                    ImVec2 p0(wp.x + x * cs, wp.y + (plan.height_cells - 1 - y) * cs);
                    dl->AddRectFilled(p0, ImVec2(p0.x + std::max(1.5f, cs), p0.y + std::max(1.5f, cs)),
                                      IM_COL32(229, 201, 131, 240));
                }
            }
        } else {
            ImGui::TextDisabled("Waiting for LiDAR sweeps...");
        }
        ImGui::End();

        // Bottom Controls Bar
        ImGui::SetNextWindowPos(ImVec2(14, static_cast<float>(fb_h) - 108.0f), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(static_cast<float>(fb_w) - 28.0f, 94.0f), ImGuiCond_Always);
        ImGui::Begin("##BottomBar", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);

        int mode_idx = (controller.acquisitionMode() == avscan::desktop::DesktopAcquisitionMode::TERRESTRIAL_STATION) ? 1 : 0;
        if (ImGui::RadioButton("Mobile SLAM", mode_idx == 0)) {
            controller.setAcquisitionMode(avscan::desktop::DesktopAcquisitionMode::MOBILE_SLAM);
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("Terrestrial Station (6x60 deg)", mode_idx == 1)) {
            controller.setAcquisitionMode(avscan::desktop::DesktopAcquisitionMode::TERRESTRIAL_STATION);
        }
        ImGui::SameLine(0, 24);
        ImGui::RadioButton("RGB", &color_mode, 0); ImGui::SameLine();
        ImGui::RadioButton("Reflectivity", &color_mode, 1); ImGui::SameLine();
        ImGui::RadioButton("Elevation Z", &color_mode, 2); ImGui::SameLine(0, 20);
        ImGui::SetNextItemWidth(100);
        ImGui::SliderFloat("Pt Size", &point_size, 1.0f, 8.0f, "%.1f");

        ImGui::Separator();
        if (controller.acquisitionMode() == avscan::desktop::DesktopAcquisitionMode::TERRESTRIAL_STATION) {
            if (!controller.isTerrestrialActive()) {
                if (ImGui::Button("New Terrestrial Station", ImVec2(200, 34))) {
                    controller.startTerrestrialStation(60.0, false);
                }
            } else {
                char step_lbl[64];
                std::snprintf(step_lbl, sizeof(step_lbl), "Next Stop %u/6 (%.0f%%)",
                              controller.currentTerrestrialStopIndex() + 1,
                              controller.terrestrialAccumulationProgress() * 100.0);
                if (ImGui::Button(step_lbl, ImVec2(200, 34))) controller.advanceToNextTerrestrialStop();
                ImGui::SameLine();
                if (ImGui::Button("Repeat Stop", ImVec2(120, 34))) controller.repeatCurrentTerrestrialStop();
                ImGui::SameLine();
                if (ImGui::Button("Finish Station", ImVec2(150, 34))) controller.finalizeAndSaveTerrestrialStation();
            }
        } else {
            if (!controller.isScanning()) {
                if (ImGui::Button("Start Scan", ImVec2(160, 34))) controller.startScanning();
            } else {
                if (ImGui::Button("Stop Scan", ImVec2(160, 34))) controller.stopScanning();
                ImGui::SameLine();
                if (!controller.isTerrestrialActive()) {
                    if (ImGui::Button("Flash 60 deg Station", ImVec2(170, 34))) {
                        controller.startTerrestrialStation(60.0, true);
                    }
                } else {
                    if (ImGui::Button("Step Flash Stop", ImVec2(140, 34))) controller.advanceToNextTerrestrialStop();
                    ImGui::SameLine();
                    if (ImGui::Button("Finish Flash", ImVec2(120, 34))) controller.finalizeAndSaveTerrestrialStation();
                }
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Save & Refine (Tier-2 Cleanup)", ImVec2(220, 34))) {
            controller.saveAndRefineSession();
            export_status = "Saved & refined session in " + controller.activeScanDir();
        }
        ImGui::SameLine();
        if (ImGui::Button("Export PLY / E57 / Bundle", ImVec2(200, 34))) {
            show_export_modal = true;
        }
        if (!export_status.empty()) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(0.29f, 0.87f, 0.50f, 1.0f), "%s", export_status.c_str());
        }
        ImGui::End();

        // Projects & Scan Manager Window
        if (show_projects_modal) {
            ImGui::Begin("Project & Scan Manager", &show_projects_modal, ImGuiWindowFlags_AlwaysAutoResize);
            ImGui::Text("New Project:");
            ImGui::InputText("Project Name", new_proj_name, sizeof(new_proj_name));
            ImGui::InputText("Site / Building", new_proj_site, sizeof(new_proj_site));
            if (ImGui::Button("Create Project")) {
                controller.createProject(new_proj_name, new_proj_site, "");
            }
            ImGui::Separator();
            ImGui::Text("New Scan in %s:", controller.activeProjectName().c_str());
            ImGui::InputText("Scan Name", new_scan_name, sizeof(new_scan_name));
            ImGui::InputText("Floor / Level", new_scan_floor, sizeof(new_scan_floor));
            if (ImGui::Button("Create & Open Scan")) {
                controller.createAndSelectScan(new_scan_name, new_scan_floor);
            }
            ImGui::Separator();
            ImGui::Text("Existing Projects:");
            for (const auto& pr : controller.listProjects()) {
                if (ImGui::Selectable((pr.name + " (" + std::to_string(pr.scan_count) + " scans)").c_str(),
                                      pr.id == controller.activeProjectId())) {
                    controller.openProject(pr.id);
                }
            }
            ImGui::End();
        }

        // Collaborative SLAM (C-SLAM) Connect Window
        if (show_cslam_window) {
            ImGui::Begin("Collaborative SLAM (C-SLAM) — Multi-Device", &show_cslam_window, ImGuiWindowFlags_AlwaysAutoResize);
            ImGui::Text("Protocol: AVCS v1 (UDP 48765 / TCP 48766)");
            ImGui::TextDisabled("Compatible: AV SLAM (iOS/Win/Linux) · AV Scan iOS · AV Scanar · AV Scan");
            bool en = controller.isCslamEnabled();
            if (ImGui::Button(en ? "Disconnect C-SLAM" : "Enable C-SLAM (Wi-Fi LAN)")) {
                controller.setCollaborativeSlamEnabled(!en);
            }
            ImGui::SameLine();
            if (ImGui::Button("Sync Shared Origin")) {
                controller.syncSharedOriginWithPeers();
            }
            ImGui::InputText("Peer IP (iPad / Jetson / PC)", peer_ip_buf, sizeof(peer_ip_buf));
            ImGui::SameLine();
            if (ImGui::Button("Connect IP")) {
                controller.connectCollaborativePeerByIp(peer_ip_buf);
            }
            ImGui::Separator();
            auto peers = controller.connectedPeers();
            ImGui::Text("Discovered Peers (%zu):", peers.size());
            for (const auto& p : peers) {
                ImGui::BulletText("%s [%s | %s] %s — %s (%u KFs)",
                                  p.display_name.c_str(),
                                  avscan::cslam::CollaborativeSlamEngine::platformToString(p.platform).c_str(),
                                  avscan::cslam::CollaborativeSlamEngine::appToString(p.app).c_str(),
                                  p.ip_address.c_str(),
                                  p.aligned ? "ALIGNED" : "SYNCING",
                                  p.keyframe_count);
            }
            ImGui::End();
        }

        // Airy Diagnostics Window
        if (show_airy_window) {
            ImGui::Begin("Airy Ethernet & Diagnostics", &show_airy_window, ImGuiWindowFlags_AlwaysAutoResize);
            ImGui::InputText("Airy IP", airy_ip_buf, sizeof(airy_ip_buf));
            ImGui::InputText("Host IP", host_ip_buf, sizeof(host_ip_buf));
            if (ImGui::Button("Run Gate 1 & Gate 2 Probe")) {
                auto cfg = controller.currentNetworkConfig();
                cfg.sensor_ip = airy_ip_buf;
                cfg.host_expected_ip = host_ip_buf;
                controller.updateNetworkConfig(cfg);
                last_diag = controller.runAiryDiagnostics();
            }
            ImGui::Separator();
            ImGui::Text("Adapter: %s | HTTP: %s (%.1f ms) | DIFOP: %s",
                        last_diag.adapter_detected ? "ONLINE" : "OFFLINE",
                        last_diag.http_reachable ? "OK" : "NO",
                        last_diag.probe_latency_ms,
                        last_diag.difop_received ? "ACTIVE" : "WAITING");
            ImGui::Text("MAC: %s | FW: %s",
                        last_diag.device_info.mac_address.c_str(),
                        last_diag.device_info.mainboard_firmware.c_str());
            ImGui::Text("MSOP: %llu | DIFOP: %llu | IMU: %llu",
                        static_cast<unsigned long long>(pkt.msop_packets_valid),
                        static_cast<unsigned long long>(pkt.difop_packets_valid),
                        static_cast<unsigned long long>(pkt.imu_packets_valid));
            ImGui::End();
        }

        // Calibration Window
        if (show_calib_window) {
            ImGui::Begin("Camera-LiDAR Extrinsic Calibration", &show_calib_window, ImGuiWindowFlags_AlwaysAutoResize);
            auto prof = controller.currentCalibrationProfile();
            auto rpy = prof.T_cam_lidar.so3.unit_quaternion().toEulerRollPitchYaw();
            ImGui::Text("Active Rig: %s", controller.activeDeviceProfileDisplayName().c_str());
            ImGui::Text("Profile ID: %s (%s)", prof.profile_id.c_str(), prof.verified ? "VERIFIED" : "SEED");
            ImGui::Text("Translation (m): [%.4f, %.4f, %.4f]", prof.T_cam_lidar.t.x, prof.T_cam_lidar.t.y, prof.T_cam_lidar.t.z);
            ImGui::Text("Euler RPY (deg): [%.2f, %.2f, %.2f]",
                        rpy.x * avscan::math::kRadToDeg,
                        rpy.y * avscan::math::kRadToDeg,
                        rpy.z * avscan::math::kRadToDeg);
            ImGui::End();
        }

        // Export Window
        if (show_export_modal) {
            ImGui::Begin("Export AV Ecosystem Package", &show_export_modal, ImGuiWindowFlags_AlwaysAutoResize);
            std::string dir = controller.activeScanDir();
            ImGui::Text("Output Directory: %s", dir.c_str());
            if (ImGui::Button("Export AV Spatial Bundle v1.1")) {
                bool ok = controller.exportSpatialBundle(dir);
                export_status = ok ? "Exported Spatial Bundle!" : "Bundle export failed.";
            }
            if (ImGui::Button("Export PLY + PCD + E57")) {
                bool ok1 = controller.exportPly((std::filesystem::path(dir) / "avscan_map.ply").string());
                bool ok2 = controller.exportPcd((std::filesystem::path(dir) / "avscan_map.pcd").string());
                bool ok3 = controller.exportE57((std::filesystem::path(dir) / "avscan_map.e57").string());
                export_status = (ok1 && ok2 && ok3) ? "Exported PLY/PCD/E57!" : "Partial export.";
            }
            ImGui::End();
        }

        ImGui::Render();
        ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
