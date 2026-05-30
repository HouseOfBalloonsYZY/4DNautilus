#include "al/app/al_App.hpp"
#include "al/io/al_Imgui.hpp"

#include "FProjection.hpp"
#include "Nautilus4D.hpp"
#include "FSlicer.hpp"

using namespace al;

struct FourDApp : public App
{
    Object4D viewer;
    Nautilus4D nautilus;

    enum class RenderMode
    {
        Projection = 0,
        Slicing = 1,
    };

    RenderMode renderMode{RenderMode::Projection};
    bool splitView{true};
    ProjectionSettings projectionSettings{};
    SliceSettings sliceSettings{};

    bool uiVisible{true};
    bool showWorldAxes{true};

    // Movement/rotation state (keyboard + UI).
    bool moveKeyPos[4]{};
    bool moveKeyNeg[4]{};
    bool moveUiPos[4]{};
    bool moveUiNeg[4]{};

    bool rotUiPos[6]{};
    bool rotUiNeg[6]{};

    const float moveSpeed = 1.5f;   // units per second
    const float rotSpeedDeg = 45.f; // degrees per second

    void onCreate() override
    {
        nav().pos(0, 0, 10);
        nav().faceToward(Vec3d(0, 0, 0));

        viewer.pos = Vec4f(0.f, 0.f, 0.f, 0.f);
        viewer.setRotation(Rotation4D::identity());

        imguiInit();
        navControl().disable();
    }

    void onAnimate(double dt) override
    {
        const float dMove = moveSpeed * static_cast<float>(dt);
        const float dRotDeg = rotSpeedDeg * static_cast<float>(dt);

        // Net movement per axis from keyboard + UI.
        for (int axis = 0; axis < 4; ++axis)
        {
            float dir = 0.f;
            if (moveKeyPos[axis] || moveUiPos[axis])
                dir += 1.f;
            if (moveKeyNeg[axis] || moveUiNeg[axis])
                dir -= 1.f;
            if (dir != 0.f)
            {
                viewer.move(axis, dir * dMove);
            }
        }

        // Rotation in 6 cardinal planes: (01,02,03,12,13,23)
        static const int planeA[6] = {0, 0, 0, 1, 1, 2};
        static const int planeB[6] = {1, 2, 3, 2, 3, 3};

        for (int i = 0; i < 6; ++i)
        {
            float dir = 0.f;
            if (rotUiPos[i])
                dir += 1.f;
            if (rotUiNeg[i])
                dir -= 1.f;
            if (dir != 0.f)
            {
                viewer.rotatePlane(planeA[i], planeB[i], dir * dRotDeg);
            }
        }

        imguiBeginFrame();
        if (uiVisible)
        {
            drawControlPanel();
        }
        imguiEndFrame();
    }

    void onDraw(Graphics &g) override
    {
        g.clear(0.1);
        g.depthTesting(true);

        if (renderMode == RenderMode::Projection)
        {
            g.viewport(0, 0, fbWidth(), fbHeight());

            drawNautilusTubeProjected(
                g,
                viewer,
                nautilus,
                nautilus.verticesLocal(),
                nautilus.hyperDistsLocal(),
                nautilus.tSteps(),
                nautilus.vSteps(),
                nautilus.projectionStartRing(),
                nautilus.projectionRingCount(),
                nautilus.drawVertexDots(),
                nautilus.pointStride(),
                nautilus.pointSize(),
                projectionSettings);

            if (showWorldAxes)
            {
                drawWorldAxes(g, viewer, projectionSettings);
            }
        }
        else
        {
            if (splitView)
            {
                // Left: projection shadow
                g.viewport(0, 0, fbWidth() / 2, fbHeight());
                drawNautilusTubeProjected(
                    g,
                    viewer,
                    nautilus,
                    nautilus.verticesLocal(),
                    nautilus.hyperDistsLocal(),
                    nautilus.tSteps(),
                    nautilus.vSteps(),
                    nautilus.projectionStartRing(),
                    nautilus.projectionRingCount(),
                    nautilus.drawVertexDots(),
                    nautilus.pointStride(),
                    nautilus.pointSize(),
                    projectionSettings);
                if (showWorldAxes)
                {
                    drawWorldAxes(g, viewer, projectionSettings);
                }

                // Right: 3D slice
                g.viewport(fbWidth() / 2, 0, fbWidth() / 2, fbHeight());
            }
            else
            {
                g.viewport(0, 0, fbWidth(), fbHeight());
            }

            std::vector<Vec4f> vertsWorld;
            std::vector<std::array<int, 4>> quads;
            nautilus.buildWorldQuadSoup(vertsWorld, quads);

            const auto slice = sliceQuadsViewerLocal(viewer, vertsWorld, quads, sliceSettings);
drawSliceResult(g, slice, sliceSettings);
        }

        // Restore default viewport for UI
        g.viewport(0, 0, fbWidth(), fbHeight());

        imguiDraw();
    }

    void drawControlPanel()
    {
        if (!uiVisible)
            return;

        ImGui::SetNextWindowBgAlpha(0.9f);
        ImGui::Begin("4D Controls");

        ImGui::Text("Toggle panel: H");
        ImGui::Separator();

        ImGui::Checkbox("Show world axes", &showWorldAxes);
        ImGui::Separator();

        ImGui::Text("Render mode");
        {
            int mode = static_cast<int>(renderMode);
            ImGui::RadioButton("Projection", &mode, static_cast<int>(RenderMode::Projection));
            ImGui::SameLine();
            ImGui::RadioButton("Slicing", &mode, static_cast<int>(RenderMode::Slicing));
            renderMode = static_cast<RenderMode>(mode);
        }

        if (renderMode == RenderMode::Slicing)
        {
            ImGui::Checkbox("Split view", &splitView);
            ImGui::SliderFloat("Slice w (viewer-local)", &sliceSettings.wPlane, -25.0f, 25.0f, "%.2f");
            ImGui::SliderFloat("Slice scale", &sliceSettings.sliceScale, 1.0f, 60.0f, "%.1f");
            ImGui::Checkbox("Slice points", &sliceSettings.drawPoints);
            ImGui::SliderFloat("Slice point size", &sliceSettings.pointSize, 1.0f, 10.0f, "%.1f");
            ImGui::Separator();
        }

        ImGui::Text("Move (click/hold or keys)");
        ImGui::Text("Keys: D/A (±X), E/Z (±Y), X/W (±Z), C/Q (±W)");

        // Movement buttons: rows per axis.
        drawAxisButtons("X", 0);
        drawAxisButtons("Y", 1);
        drawAxisButtons("Z", 2);
        drawAxisButtons("W", 3);

        ImGui::Separator();
        ImGui::Text("Rotate planes (click/hold)");

        const char *planeLabels[6] = {"XY", "XZ", "XW", "YZ", "YW", "ZW"};
        for (int i = 0; i < 6; ++i)
        {
            ImGui::PushID(i);
            ImGui::Text("%s", planeLabels[i]);
            ImGui::SameLine();
            if (ImGui::Button("-"))
            {
                // Press event not used; continuous handled via IsItemActive().
            }
            rotUiNeg[i] = ImGui::IsItemActive();
            ImGui::SameLine();
            if (ImGui::Button("+"))
            {
            }
            rotUiPos[i] = ImGui::IsItemActive();
            ImGui::PopID();
        }

        nautilus.drawImGuiControls();

        ImGui::End();
    }

    void drawAxisButtons(const char *label, int axis)
    {
        ImGui::PushID(axis);
        ImGui::Text("%s", label);
        ImGui::SameLine();
        if (ImGui::Button("-"))
        {
        }
        moveUiNeg[axis] = ImGui::IsItemActive();
        ImGui::SameLine();
        if (ImGui::Button("+"))
        {
        }
        moveUiPos[axis] = ImGui::IsItemActive();
        ImGui::PopID();
    }

    bool onKeyDown(const Keyboard &k) override
    {
        const int key = k.key();

        // Toggle UI visibility.
        if (key == 'h' || key == 'H')
        {
            uiVisible = !uiVisible;
            return true;
        }

        // Movement keys:
        //   D = +x, A = -x
        //   E = +y, Z = -y
        //   X = +z, W = -z
        //   C = +w, Q = -w
        switch (key)
        {
        case 'd':
        case 'D':
            moveKeyPos[0] = true;
            return true;
        case 'a':
        case 'A':
            moveKeyNeg[0] = true;
            return true;
        case 'e':
        case 'E':
            moveKeyPos[1] = true;
            return true;
        case 'z':
        case 'Z':
            moveKeyNeg[1] = true;
            return true;
        case 'x':
        case 'X':
            moveKeyPos[2] = true;
            return true;
        case 'w':
        case 'W':
            moveKeyNeg[2] = true;
            return true;
        case 'c':
        case 'C':
            moveKeyPos[3] = true;
            return true;
        case 'q':
        case 'Q':
            moveKeyNeg[3] = true;
            return true;
        default:
            break;
        }

        // Reset
        if (key == ' ')
        {
            viewer.pos = Vec4f(0.f, 0.f, 0.f, 0.f);
            viewer.setRotation(Rotation4D::identity());

            // Clear all movement / rotation flags so reset is stable.
            for (int i = 0; i < 4; ++i)
            {
                moveKeyPos[i] = moveKeyNeg[i] = false;
                moveUiPos[i] = moveUiNeg[i] = false;
            }
            for (int i = 0; i < 6; ++i)
            {
                rotUiPos[i] = rotUiNeg[i] = false;
            }

            return true;
        }

        return false;
    }

    bool onKeyUp(const Keyboard &k) override
    {
        const int key = k.key();
        switch (key)
        {
        case 'd':
        case 'D':
            moveKeyPos[0] = false;
            return true;
        case 'a':
        case 'A':
            moveKeyNeg[0] = false;
            return true;
        case 'e':
        case 'E':
            moveKeyPos[1] = false;
            return true;
        case 'z':
        case 'Z':
            moveKeyNeg[1] = false;
            return true;
        case 'x':
        case 'X':
            moveKeyPos[2] = false;
            return true;
        case 'w':
        case 'W':
            moveKeyNeg[2] = false;
            return true;
        case 'c':
        case 'C':
            moveKeyPos[3] = false;
            return true;
        case 'q':
        case 'Q':
            moveKeyNeg[3] = false;
            return true;
        default:
            break;
        }
        return false;
    }

    void onExit() override
    {
        imguiShutdown();
    }
};

int main()
{
    FourDApp app;
    app.fps(60);
    app.start();
    return 0;
}
