#include "al/app/al_App.hpp"
#include "al/io/al_Imgui.hpp"
#include "al/graphics/al_Shapes.hpp"
// #include "al/io/al_Window.hpp"
#include "4DNautilusHypervolume.hpp"
#include "Fslicer.hpp"


using namespace al;

struct FourDApp : public App
{
	object4D viewer;
	NautilusHypervolume4D nautilus;

	enum class RenderMode
	{
		Projection = 0,
		Slicing = 1,
	};

	RenderMode renderMode{RenderMode::Projection};
	bool splitView{true};
	HyperSliceSettings sliceSettings{};

	bool uiVisible{true};
	bool showWorldAxes{true};

	// Movement/rotation state (keyboard + UI).
	bool moveKeyPos[4]{};
	bool moveKeyNeg[4]{};
	bool moveUiPos[4]{};
	bool moveUiNeg[4]{};

	bool rotKeyPos[6]{};
	bool rotKeyNeg[6]{};
	bool rotUiPos[6]{};
	bool rotUiNeg[6]{};

	const float moveSpeed = 1.5f;     // units per second
	const float rotSpeedDeg = 45.f;   // degrees per second

	void onCreate() override
	{
		nav().pos(0, 0, 10);
		nav().faceToward(Vec3d(0, 0, 0));

		viewer.pos = Vec4f(0.f, 0.f, 0.f, 0.f);
		viewer.setRotation(Rotation4D::identity());

		imguiInit();

		// Disable Allolib default nav keys; we drive viewer + nav explicitly below.
		// navControl().disable();
	}

	void onAnimate(double dt) override
	{
		const float dMove = moveSpeed * static_cast<float>(dt);
		const float dRotDeg = rotSpeedDeg * static_cast<float>(dt);

		// Net movement per axis from keyboard + UI.
		for (int axis = 0; axis < 4; ++axis)
		{
			float dir = 0.f;
			if (moveKeyPos[axis] || moveUiPos[axis]) dir += 1.f;
			if (moveKeyNeg[axis] || moveUiNeg[axis]) dir -= 1.f;
			if (dir != 0.f)
			{
				viewer.move(axis, dir * dMove);
				// Keep 3D nav in sync for x,y,z (Allolib camera convention).
				if (axis < 3)
				{
					nav().pos()[axis] += dir * dMove;
				}
			}
		}

		// Rotation in 6 cardinal planes: XY, XZ, XW, YZ, YW, ZW
		static const int planeA[6] = {0, 0, 0, 1, 1, 2};
		static const int planeB[6] = {1, 2, 3, 2, 3, 3};

		for (int i = 0; i < 6; ++i)
		{
			float dir = 0.f;
			if (rotKeyPos[i] || rotUiPos[i]) dir += 1.f;
			if (rotKeyNeg[i] || rotUiNeg[i]) dir -= 1.f;
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

	void onDraw(Graphics& g) override
	{
		g.clear(0.1);
		g.depthTesting(true);

		if (renderMode == RenderMode::Projection)
		{
			g.viewport(0, 0, fbWidth(), fbHeight());

			nautilus.drawProjectedShadow(g, viewer);

			if (showWorldAxes)
			{
				drawWorldAxes(g, viewer);
			}
		}
		else
		{
			if (splitView)
			{
				// Left: projection shadow
				g.viewport(0, 0, fbWidth() / 2, fbHeight());
				nautilus.drawProjectedShadow(g, viewer);
				if (showWorldAxes)
				{
					drawWorldAxes(g, viewer);
				}

				// Right: 3D slice
				g.viewport(fbWidth() / 2, 0, fbWidth() / 2, fbHeight());
			}
			else
			{
				g.viewport(0, 0, fbWidth(), fbHeight());
			}

			std::vector<Vec4f> vertsWorld;
			std::vector<std::array<int, 5>> simplices;
			nautilus.buildWorldSimplices(vertsWorld, simplices);

			const auto slice = slice4SimplicesViewerLocal(viewer, vertsWorld, simplices, sliceSettings);
			drawHyperSlice(g, slice, sliceSettings);
		}

		// Restore default viewport for UI
		g.viewport(0, 0, fbWidth(), fbHeight());

		imguiDraw();
	}

	void drawControlPanel()
	{
		if (!uiVisible) return;

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
			ImGui::Checkbox("Slice edges", &sliceSettings.drawEdges);
			ImGui::Checkbox("Slice points", &sliceSettings.drawPoints);
			ImGui::SliderFloat("Slice point size", &sliceSettings.pointSize, 1.0f, 10.0f, "%.1f");
			ImGui::Separator();
		}

		ImGui::Text("Move (click/hold or keys, Allolib XYZ)");
		ImGui::Text("  D/A = +/-X   E/C = +/-Y   W/X = forward/back (-Z/+Z)");
		ImGui::Text("  R/V = kata/ana (+W/-W, toward/away in 4D)");

		// Movement buttons: rows per axis.
		drawAxisButtons("X", 0);
		drawAxisButtons("Y", 1);
		drawAxisButtons("Z", 2);
		drawAxisButtons("W (4D)", 3);

		ImGui::Separator();
		ImGui::Text("Rotate planes (click/hold or keys)");
		ImGui::Text("  Q/Z = XY   arrows L/R = XZ   arrows U/D = YZ");
		ImGui::Text("  1/2 = XW   3/4 = YW   5/6 = ZW");

		const char* planeLabels[6] = {"XY", "XZ", "XW", "YZ", "YW", "ZW"};
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

	void drawAxisButtons(const char* label, int axis)
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

	bool onKeyDown(const Keyboard& k) override
	{
		const int key = k.key();

		// Toggle UI visibility.
		if (key == 'h' || key == 'H')
		{
			uiVisible = !uiVisible;
			return true;
		}

		// Translation (Allolib XYZ + R/V for 4D w)
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
		case 'c':
		case 'C':
			moveKeyNeg[1] = true;
			return true;
		case 'w':
		case 'W':
			moveKeyNeg[2] = true; // forward = -z (Allolib)
			return true;
		case 'x':
		case 'X':
			moveKeyPos[2] = true; // back = +z
			return true;
		case 'r':
		case 'R':
			moveKeyPos[3] = true; // +w = kata (toward viewer)
			return true;
		case 'v':
		case 'V':
			moveKeyNeg[3] = true; // -w = ana (away from viewer)
			return true;

		// Rotation (Allolib spatial planes + 1-6 for w planes)
		case 'q':
		case 'Q':
			rotKeyPos[0] = true; // XY (bank)
			return true;
		case 'z':
		case 'Z':
			rotKeyNeg[0] = true;
			return true;
		case Keyboard::LEFT:
			rotKeyPos[1] = true; // XZ (azimuth)
			return true;
		case Keyboard::RIGHT:
			rotKeyNeg[1] = true;
			return true;
		case Keyboard::UP:
			rotKeyPos[3] = true; // YZ (elevation)
			return true;
		case Keyboard::DOWN:
			rotKeyNeg[3] = true;
			return true;
		case '1':
			rotKeyPos[2] = true; // XW
			return true;
		case '2':
			rotKeyNeg[2] = true;
			return true;
		case '3':
			rotKeyPos[4] = true; // YW
			return true;
		case '4':
			rotKeyNeg[4] = true;
			return true;
		case '5':
			rotKeyPos[5] = true; // ZW
			return true;
		case '6':
			rotKeyNeg[5] = true;
			return true;
		default:
			break;
		}

		// Reset viewer + 3D nav
		if (key == ' ')
		{
			viewer.pos = Vec4f(0.f, 0.f, 0.f, 0.f);
			viewer.setRotation(Rotation4D::identity());
			nav().halt();
			nav().pos(0, 0, 10);
			nav().faceToward(Vec3d(0, 0, 0));

			for (int i = 0; i < 4; ++i)
			{
				moveKeyPos[i] = moveKeyNeg[i] = false;
				moveUiPos[i] = moveUiNeg[i] = false;
			}
			for (int i = 0; i < 6; ++i)
			{
				rotKeyPos[i] = rotKeyNeg[i] = false;
				rotUiPos[i] = rotUiNeg[i] = false;
			}

			return true;
		}

		return false;
	}

	bool onKeyUp(const Keyboard& k) override
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
		case 'c':
		case 'C':
			moveKeyNeg[1] = false;
			return true;
		case 'w':
		case 'W':
			moveKeyNeg[2] = false;
			return true;
		case 'x':
		case 'X':
			moveKeyPos[2] = false;
			return true;
		case 'r':
		case 'R':
			moveKeyPos[3] = false;
			return true;
		case 'v':
		case 'V':
			moveKeyNeg[3] = false;
			return true;

		case 'q':
		case 'Q':
			rotKeyPos[0] = false;
			return true;
		case 'z':
		case 'Z':
			rotKeyNeg[0] = false;
			return true;
		case Keyboard::LEFT:
			rotKeyPos[1] = false;
			return true;
		case Keyboard::RIGHT:
			rotKeyNeg[1] = false;
			return true;
		case Keyboard::UP:
			rotKeyPos[3] = false;
			return true;
		case Keyboard::DOWN:
			rotKeyNeg[3] = false;
			return true;
		case '1':
			rotKeyPos[2] = false;
			return true;
		case '2':
			rotKeyNeg[2] = false;
			return true;
		case '3':
			rotKeyPos[4] = false;
			return true;
		case '4':
			rotKeyNeg[4] = false;
			return true;
		case '5':
			rotKeyPos[5] = false;
			return true;
		case '6':
			rotKeyNeg[5] = false;
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




