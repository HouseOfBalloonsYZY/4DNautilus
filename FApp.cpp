#include "al/app/al_App.hpp"
#include "al/io/al_Imgui.hpp"
#include "al/graphics/al_Shapes.hpp"

#include "FProjectorPlane.hpp"
#include "FSlicer.hpp"
#include "Camera4D.hpp"
#include "Nautilus4D.hpp"
#include "Scene4D.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

using namespace al;

namespace
{

float clamp01(float x)
{
	return std::max(0.f, std::min(1.f, x));
}

al::Color nautilusGradientColor(float t)
{
	const al::Color inner(1.f, 1.f, 1.f, 1.f);
	const al::Color outer(1.f, 100.f / 255.f, 0.f, 76.f / 255.f);

	al::Color c;
	c.r = inner.r + t * (outer.r - inner.r);
	c.g = inner.g + t * (outer.g - inner.g);
	c.b = inner.b + t * (outer.b - inner.b);
	c.a = inner.a + t * (outer.a - inner.a);
	return c;
}

} // namespace

struct FourDApp : public App
{
	Camera4D camera4D;
	Nautilus4D nautilus;
	Mesh4D nautilusMesh;
	Scene4D scene;

	enum class RenderMode
	{
		Projection = 0,
		Slicing = 1,
	};

	RenderMode renderMode{RenderMode::Projection};
	bool splitView{true};
	ProjectionSettings projectionSettings{};
	Slicer4D::Settings sliceSettings{};

	bool uiVisible{true};
	bool showWorldAxes{true};

	// Input tuning (maps keys/UI to target rates on camera4D; integration lives on Object4D).
	static constexpr float kMoveSpeed = 1.5f;
	static constexpr float kRotSpeedDeg = 45.f;

	void onCreate() override
	{
		camera4D.halt();
		camera4D.home();

		nav().pos(0, 0, 0);
		nav().faceToward(Vec3d(0, 0, -1));

		imguiInit();
		navControl().disable();

		nautilus.publishVolumeMesh(nautilusMesh);
		scene.add(nautilusMesh);
	}

	void onAnimate(double dt) override
	{
		const float rotSpeedRad = kRotSpeedDeg * (3.14159265358979f / 180.f);

		imguiBeginFrame();

		// UI is polled each frame; add its target rates only for this step (keyboard
		// rates persist on camera4D via add/sub in onKeyDown/onKeyUp, Nav-style).
		Vec4f uiMoveSpeedLocal{0.f, 0.f, 0.f, 0.f};
		std::array<float, 6> uiRotateSpeedLocal{};

		if (uiVisible)
		{
			drawControlPanel(uiMoveSpeedLocal, uiRotateSpeedLocal, rotSpeedRad);
		}

		camera4D.addMoveSpeedLocal(uiMoveSpeedLocal);
		camera4D.addRotateSpeedLocal(uiRotateSpeedLocal);
		camera4D.step(dt);
		camera4D.addMoveSpeedLocal(-uiMoveSpeedLocal);
		camera4D.addRotateSpeedLocal(negatePlaneRates(uiRotateSpeedLocal));

		imguiEndFrame();
	}

	void onDraw(Graphics &g) override
	{
		g.clear(0.1);
		g.depthTesting(true);

		nautilus.publishVolumeMesh(nautilusMesh);
		scene.prepare(camera4D);

		ProjectorPlane projector(projectionSettings);

		if (renderMode == RenderMode::Projection)
		{
			g.viewport(0, 0, fbWidth(), fbHeight());
			drawSceneProjection(g, projector);

			if (showWorldAxes)
			{
				projector.drawWorldAxes(g, camera4D);
			}
		}
		else
		{
			if (splitView)
			{
				g.viewport(0, 0, fbWidth() / 2, fbHeight());
				drawSceneProjection(g, projector);
				if (showWorldAxes)
				{
					projector.drawWorldAxes(g, camera4D);
				}

				g.viewport(fbWidth() / 2, 0, fbWidth() / 2, fbHeight());
			}
			else
			{
				g.viewport(0, 0, fbWidth(), fbHeight());
			}

			drawSceneSlice(g);
		}

		g.viewport(0, 0, fbWidth(), fbHeight());
		imguiDraw();
	}

	static std::array<float, 6> negatePlaneRates(const std::array<float, 6> &rates)
	{
		std::array<float, 6> out = rates;
		for (float &r : out)
		{
			r = -r;
		}
		return out;
	}

	void drawSceneProjection(Graphics &g, ProjectorPlane &projector)
	{
		Mesh lines;
		lines.primitive(Mesh::LINES);
		g.blending(true);
		g.blendTrans();

		for (Mesh4D *mesh : scene.meshes())
		{
			const std::vector<Vec4f> &local = mesh->verticesLocal();
			const std::vector<Vec4f> &world = mesh->verticesInWorld();
			float minD = std::numeric_limits<float>::max();
			float maxD = 0.f;
			for (const Vec4f &v : local)
			{
				const float d = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z + v.w * v.w);
				minD = std::min(minD, d);
				maxD = std::max(maxD, d);
			}
			if (maxD <= minD)
			{
				maxD = minD + 1.f;
			}

			for (const Edge4D &edge : mesh->elementsData().edges)
			{
				const size_t ia = static_cast<size_t>(edge.first);
				const size_t ib = static_cast<size_t>(edge.second);
				Vec3f a;
				Vec3f b;
				if (!projector.tryProjectWorld(camera4D, world[ia], a))
				{
					continue;
				}
				if (!projector.tryProjectWorld(camera4D, world[ib], b))
				{
					continue;
				}
				const float da = std::sqrt(local[ia].x * local[ia].x + local[ia].y * local[ia].y
					+ local[ia].z * local[ia].z + local[ia].w * local[ia].w);
				const float db = std::sqrt(local[ib].x * local[ib].x + local[ib].y * local[ib].y
					+ local[ib].z * local[ib].z + local[ib].w * local[ib].w);
				lines.color(nautilusGradientColor(clamp01((da - minD) / (maxD - minD))));
				lines.vertex(a);
				lines.color(nautilusGradientColor(clamp01((db - minD) / (maxD - minD))));
				lines.vertex(b);
			}
		}

		projector.drawLineMesh(g, lines);
		g.blending(false);
	}

	void drawSceneSlice(Graphics &g)
	{
		for (Mesh4D *mesh : scene.meshes())
		{
			Slicer4D slicer(sliceSettings);
			slicer.setVerticesWorld(mesh->verticesInWorld());
			for (const Pentachoron4D &cell : mesh->elementsData().pentachora)
			{
				slicer.add4Cell(cell.i);
			}
			for (const Tetrahedron4D &tet : mesh->elementsData().tetrahedra)
			{
				slicer.add3Simplex(tet.i);
			}
			for (const Triangle4D &tri : mesh->elementsData().triangles)
			{
				slicer.add2Triangle(tri.i);
			}
			for (const Edge4D &edge : mesh->elementsData().edges)
			{
				slicer.add1Edge(edge.first, edge.second);
			}
			const Slicer4D::Result result = slicer.slice(camera4D);
			slicer.draw(g, result);
		}
	}

	void drawControlPanel(
		Vec4f &uiMoveSpeedLocal,
		std::array<float, 6> &uiRotateSpeedLocal,
		float rotSpeedRad)
	{
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
		ImGui::Checkbox("Split view", &splitView);

		if (renderMode == RenderMode::Projection
			|| (renderMode == RenderMode::Slicing && splitView))
		{
			ImGui::Text("Projection (orange/white tube)");
			int mode = static_cast<int>(projectionSettings.mode);
			ImGui::RadioButton("1-point", &mode, static_cast<int>(Perspective::OnePoint));
			ImGui::SameLine();
			ImGui::RadioButton("2-point (x)", &mode, static_cast<int>(Perspective::TwoPoint));
			projectionSettings.mode = static_cast<Perspective>(mode);
			ImGui::SliderFloat("Vanish X distance", &projectionSettings.vanishX, 4.f, 80.f, "%.1f");
			ImGui::Separator();
		}

		if (renderMode == RenderMode::Slicing)
		{
			ImGui::SliderFloat("Slice w (viewer-local)", &sliceSettings.wPlane, -25.f, 25.f, "%.2f");
			ImGui::SliderFloat("Slice scale", &sliceSettings.sliceScale, 1.f, 60.f, "%.1f");
			ImGui::Checkbox("Slice solid", &sliceSettings.drawVolume);
			ImGui::Checkbox("Slice points", &sliceSettings.drawPoints);
			ImGui::SliderFloat("Slice point size", &sliceSettings.pointSize, 1.f, 10.f, "%.1f");
			ImGui::Separator();
		}

		ImGui::Text("Move (click/hold or keys, viewer-local face axes)");
		ImGui::Text("  D/A = +/-X   E/C = +/-Y   W/X = forward/back (-Z/+Z)");
		ImGui::Text("  R/V = kata/ana (+W/-W)");

		drawMoveAxisButtons("X", 0, uiMoveSpeedLocal);
		drawMoveAxisButtons("Y", 1, uiMoveSpeedLocal);
		drawMoveAxisButtons("Z", 2, uiMoveSpeedLocal);
		drawMoveAxisButtons("W (4D)", 3, uiMoveSpeedLocal);

		ImGui::Separator();
		ImGui::Text("Rotate planes (click/hold or keys)");
		ImGui::Text("  Q/Z = XY   arrows L/R = XZ   arrows U/D = YZ");
		ImGui::Text("  1/2 = XW   3/4 = YW   5/6 = ZW");

		static const char *planeLabels[6] = {"XY", "XZ", "XW", "YZ", "YW", "ZW"};
		for (int i = 0; i < 6; ++i)
		{
			drawRotatePlaneButtons(planeLabels[i], i, uiRotateSpeedLocal, rotSpeedRad);
		}

		nautilus.drawImGuiControls();

		ImGui::End();
	}

	void drawMoveAxisButtons(const char *label, int axis, Vec4f &moveSpeedLocal)
	{
		ImGui::PushID(axis);
		ImGui::Text("%s", label);
		ImGui::SameLine();
		if (ImGui::Button("-"))
		{
		}
		if (ImGui::IsItemActive())
		{
			moveSpeedLocal[static_cast<size_t>(axis)] -= kMoveSpeed;
		}
		ImGui::SameLine();
		if (ImGui::Button("+"))
		{
		}
		if (ImGui::IsItemActive())
		{
			moveSpeedLocal[static_cast<size_t>(axis)] += kMoveSpeed;
		}
		ImGui::PopID();
	}

	void drawRotatePlaneButtons(
		const char *label,
		int plane,
		std::array<float, 6> &rotateSpeedLocal,
		float rotSpeedRad)
	{
		ImGui::PushID(plane);
		ImGui::Text("%s", label);
		ImGui::SameLine();
		if (ImGui::Button("-"))
		{
		}
		if (ImGui::IsItemActive())
		{
			rotateSpeedLocal[static_cast<size_t>(plane)] -= rotSpeedRad;
		}
		ImGui::SameLine();
		if (ImGui::Button("+"))
		{
		}
		if (ImGui::IsItemActive())
		{
			rotateSpeedLocal[static_cast<size_t>(plane)] += rotSpeedRad;
		}
		ImGui::PopID();
	}

	void resetNavigationInput()
	{
		camera4D.halt();
		camera4D.home();
		nav().pos(0, 0, 0);
		nav().faceToward(Vec3d(0, 0, -1));
	}

	bool onKeyDown(const Keyboard &k) override
	{
		const int key = k.key();
		const float rotSpeedRad = kRotSpeedDeg * (3.14159265358979f / 180.f);

		if (key == 'h' || key == 'H')
		{
			uiVisible = !uiVisible;
			return true;
		}

		if (key == ' ')
		{
			resetNavigationInput();
			return true;
		}

		switch (key)
		{
        //X
		case 'd':
		case 'D':
			camera4D.addMoveSpeedLocal(0, kMoveSpeed);
			return true;
		case 'a':
		case 'A':
			camera4D.addMoveSpeedLocal(0, -kMoveSpeed);
			return true;
        // Y
		case 'e':
		case 'E':
			camera4D.addMoveSpeedLocal(1, kMoveSpeed);
			return true;
		case 'c':
		case 'C':
			camera4D.addMoveSpeedLocal(1, -kMoveSpeed);
			return true;
        // Z
		case 'w':
		case 'W':
			camera4D.addMoveSpeedLocal(2, kMoveSpeed);
			return true;
		case 'x':
		case 'X':
			camera4D.addMoveSpeedLocal(2, -kMoveSpeed);
			return true;
        // W
		case 'r':
		case 'R':
			camera4D.addMoveSpeedLocal(3, kMoveSpeed);
			return true;
		case 'v':
		case 'V':
			camera4D.addMoveSpeedLocal(3, -kMoveSpeed);
			return true;

        // XY 
        case 'z':
		case 'Z':
			camera4D.addRotateSpeedLocal(0, rotSpeedRad);
			return true;   
		case 'q':
		case 'Q':
			camera4D.addRotateSpeedLocal(0, -rotSpeedRad);
			return true;
        // XZ
        case Keyboard::RIGHT:
			camera4D.addRotateSpeedLocal(1, rotSpeedRad);
            return true;
		case Keyboard::LEFT:
			camera4D.addRotateSpeedLocal(1, -rotSpeedRad);
			return true;
        // ZW
		case '1':
			camera4D.addRotateSpeedLocal(2, rotSpeedRad);
			return true;
		case '2':
			camera4D.addRotateSpeedLocal(2, -rotSpeedRad);
			return true;
        // YZ
		case Keyboard::UP:
			camera4D.addRotateSpeedLocal(3, rotSpeedRad);
			return true;
		case Keyboard::DOWN:
			camera4D.addRotateSpeedLocal(3, -rotSpeedRad);
			return true;
        // YW
		case '3':
			camera4D.addRotateSpeedLocal(4, rotSpeedRad);
			return true;
		case '4':
			camera4D.addRotateSpeedLocal(4, -rotSpeedRad);
			return true;
        // ZW
		case '5':
			camera4D.addRotateSpeedLocal(5, rotSpeedRad);
			return true;
		case '6':
			camera4D.addRotateSpeedLocal(5, -rotSpeedRad);
			return true;
		default:
			break;
		}

		return false;
	}

	bool onKeyUp(const Keyboard &k) override
	{
		const int key = k.key();
		const float rotSpeedRad = kRotSpeedDeg * (3.14159265358979f / 180.f);

		switch (key)
		{
		//X
		case 'd':
		case 'D':
			camera4D.addMoveSpeedLocal(0, -kMoveSpeed);
			return true;
		case 'a':
		case 'A':
			camera4D.addMoveSpeedLocal(0, kMoveSpeed);
			return true;
        // Y
		case 'e':
		case 'E':
			camera4D.addMoveSpeedLocal(1, -kMoveSpeed);
			return true;
		case 'c':
		case 'C':
			camera4D.addMoveSpeedLocal(1, kMoveSpeed);
			return true;
        // Z
		case 'w':
		case 'W':
			camera4D.addMoveSpeedLocal(2, -kMoveSpeed);
			return true;
		case 'x':
		case 'X':
			camera4D.addMoveSpeedLocal(2, kMoveSpeed);
			return true;
        // W
		case 'r':
		case 'R':
			camera4D.addMoveSpeedLocal(3, -kMoveSpeed);
			return true;
		case 'v':
		case 'V':
			camera4D.addMoveSpeedLocal(3, kMoveSpeed);
			return true;

        // XY 
        case 'z':
		case 'Z':
			camera4D.addRotateSpeedLocal(0, -rotSpeedRad);
			return true;   
		case 'q':
		case 'Q':
			camera4D.addRotateSpeedLocal(0, rotSpeedRad);
			return true;
        // XZ
        case Keyboard::RIGHT:
			camera4D.addRotateSpeedLocal(1, -rotSpeedRad);
            return true;
		case Keyboard::LEFT:
			camera4D.addRotateSpeedLocal(1, rotSpeedRad);
			return true;
        // ZW
		case '1':
			camera4D.addRotateSpeedLocal(2, -rotSpeedRad);
			return true;
		case '2':
			camera4D.addRotateSpeedLocal(2, rotSpeedRad);
			return true;
        // YZ
		case Keyboard::UP:
			camera4D.addRotateSpeedLocal(3, -rotSpeedRad);
			return true;
		case Keyboard::DOWN:
			camera4D.addRotateSpeedLocal(3, rotSpeedRad);
			return true;
        // YW
		case '3':
			camera4D.addRotateSpeedLocal(4, -rotSpeedRad);
			return true;
		case '4':
			camera4D.addRotateSpeedLocal(4, rotSpeedRad);
			return true;
        // ZW
		case '5':
			camera4D.addRotateSpeedLocal(5, -rotSpeedRad);
			return true;
		case '6':
			camera4D.addRotateSpeedLocal(5, rotSpeedRad);
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
