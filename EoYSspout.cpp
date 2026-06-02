#include "al/app/al_App.hpp"
#include "al/io/al_Imgui.hpp"
#include "al/graphics/al_FBO.hpp"
#include "al/graphics/al_Shapes.hpp"

#include "4DNautilusHypervolume.hpp"
#include "FProjectorPlane.hpp"
#include "FSlicer.hpp"
#include "Nav4D.hpp"
#include "Nautilus4D.hpp"

#ifdef _WIN32
#include "SpoutSender.h"
#endif

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

struct EoYSspout : public App
{
	static constexpr const char *kSpoutSenderName = "4DNautilus";
	static constexpr int kSpoutWidth = 10185;
	static constexpr int kSpoutHeight = 2160;

	Nav4D camera4D = Nav4D(Vec4f(0.f, 0.f, 3.f, 0.f), Rotation4D::identity());
	Nautilus4D nautilus;
	NautilusHypervolume4D hypervolume;

	FBO sceneFbo;
	Texture sceneColorTex;
	RBO sceneDepthRbo;

#ifdef _WIN32
	SpoutSender spoutSender;
#endif

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
	bool spoutEnabled{true};

	static constexpr float kMoveSpeed = 1.5f;
	static constexpr float kRotSpeedDeg = 45.f;

	void updateSceneFBO(int w, int h)
	{
		if (w <= 0 || h <= 0)
		{
			return;
		}

		const unsigned uw = static_cast<unsigned>(w);
		const unsigned uh = static_cast<unsigned>(h);

		sceneColorTex.create2D(uw, uh);
		sceneDepthRbo.resize(uw, uh);

		sceneFbo.bind();
		sceneFbo.attachTexture2D(sceneColorTex);
		sceneFbo.attachRBO(sceneDepthRbo);
		sceneFbo.unbind();
	}

	void onCreate() override
	{
		camera4D.halt();
		camera4D.home();

		nav().pos(0, 0, 0);
		nav().faceToward(Vec3d(0, 0, -1));

		imguiInit();
		navControl().disable();

		updateSceneFBO(kSpoutWidth, kSpoutHeight);

#ifdef _WIN32
		spoutSender.SetSenderName(kSpoutSenderName);
#endif
	}

	void onResize(int /*w*/, int /*h*/) override
	{
		// Spout/FBO output stays at kSpoutWidth x kSpoutHeight; window preview scales independently.
	}

	void onAnimate(double dt) override
	{
		const float rotSpeedRad = kRotSpeedDeg * (3.14159265358979f / 180.f);

		imguiBeginFrame();

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

	void drawScene(Graphics &g, int viewW, int viewH)
	{
		g.depthTesting(true);

		ProjectorPlane projector(projectionSettings);

		if (renderMode == RenderMode::Projection)
		{
			g.viewport(0, 0, viewW, viewH);
			drawNautilusProjection(g, projector);

			if (showWorldAxes)
			{
				projector.drawWorldAxes(g, camera4D);
			}
		}
		else
		{
			if (splitView)
			{
				g.viewport(0, 0, viewW / 2, viewH);
				drawNautilusProjection(g, projector);
				if (showWorldAxes)
				{
					projector.drawWorldAxes(g, camera4D);
				}

				g.viewport(viewW / 2, 0, viewW / 2, viewH);
			}
			else
			{
				g.viewport(0, 0, viewW, viewH);
			}

			std::vector<Vec4f> vertsWorld;
			std::vector<std::array<int, 5>> simplices;
			hypervolume.buildWorldSimplices(vertsWorld, simplices);

			const HyperSliceResult slice = slice4SimplicesViewerLocal(
				camera4D,
				vertsWorld,
				simplices,
				sliceSettings);
			drawHyperSlice(g, slice, sliceSettings);
		}
	}

	void onDraw(Graphics &g) override
	{
		const int outW = kSpoutWidth;
		const int outH = kSpoutHeight;

		if (sceneColorTex.width() != static_cast<unsigned>(outW)
			|| sceneColorTex.height() != static_cast<unsigned>(outH))
		{
			updateSceneFBO(outW, outH);
		}

		if (outW > 0 && outH > 0 && sceneColorTex.id() != 0)
		{
			sceneFbo.bind();
			g.viewport(0, 0, outW, outH);
			g.clear(0.1);
			drawScene(g, outW, outH);
			sceneFbo.unbind();

#ifdef _WIN32
			if (spoutEnabled && sceneColorTex.id() != 0)
			{
				spoutSender.SendTexture(
					static_cast<GLuint>(sceneColorTex.id()),
					GL_TEXTURE_2D,
					static_cast<unsigned int>(outW),
					static_cast<unsigned int>(outH),
					true,
					0);
			}
#endif
		}

		g.viewport(0, 0, fbWidth(), fbHeight());
		g.clear(0.1);

		if (sceneColorTex.id() != 0)
		{
			g.quadViewport(sceneColorTex);
		}

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

	void drawControlPanel(
		Vec4f &uiMoveSpeedLocal,
		std::array<float, 6> &uiRotateSpeedLocal,
		float rotSpeedRad)
	{
		ImGui::SetNextWindowBgAlpha(0.9f);
		ImGui::Begin("4D Controls");

		ImGui::Text("Toggle panel: H");
		ImGui::Separator();

#ifdef _WIN32
		ImGui::Text("Spout sender: %s", kSpoutSenderName);
		ImGui::Text("Spout output: %d x %d", kSpoutWidth, kSpoutHeight);
		ImGui::Checkbox("Send Spout output", &spoutEnabled);
		if (spoutSender.IsInitialized())
		{
			ImGui::Text(
				"Spout: %ux%u",
				spoutSender.GetWidth(),
				spoutSender.GetHeight());
		}
		ImGui::Separator();
#else
		ImGui::Text("Spout: Windows build only");
		ImGui::Separator();
#endif

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
			ImGui::Checkbox("Split view", &splitView);
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

		if (renderMode == RenderMode::Projection
			|| (renderMode == RenderMode::Slicing && splitView))
		{
			nautilus.drawImGuiControls();
		}

		if (renderMode == RenderMode::Slicing)
		{
			ImGui::Separator();
			ImGui::Text("Hypervolume (slicing)");
			hypervolume.drawImGuiControls();
		}

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

	void drawNautilusProjection(Graphics &g, const ProjectorPlane &projector)
	{
		const std::vector<Vec4f> &verticesLocal = nautilus.verticesLocal();
		const std::vector<float> &hyperDists = nautilus.hyperDistsLocal();
		const int tSteps = nautilus.tSteps();
		const int vSteps = nautilus.vSteps();
		const int startRing = nautilus.projectionStartRing();
		const int ringCount = nautilus.projectionRingCount();

		if (verticesLocal.empty() || tSteps <= 0 || vSteps <= 0)
		{
			return;
		}

		const int ringCountClamped = std::max(1, std::min(ringCount, tSteps));
		const int startRingClamped = std::max(0, std::min(startRing, tSteps - 1));
		const int ringsForProj = ringCountClamped + 1;
		const size_t windowVerts = static_cast<size_t>(ringsForProj) * static_cast<size_t>(vSteps);

		std::vector<Vec3f> proj(windowVerts);
		std::vector<float> dists(windowVerts);
		std::vector<bool> visible(windowVerts, false);

		float minD = std::numeric_limits<float>::max();
		float maxD = std::numeric_limits<float>::lowest();

		for (int k = 0; k <= ringCountClamped; ++k)
		{
			const int r = (startRingClamped + k) % tSteps;

			for (int v = 0; v < vSteps; ++v)
			{
				const int localIdx = r * vSteps + v;
				const size_t winIdx = static_cast<size_t>(k * vSteps + v);

				const Vec4f vWorld = ProjectorPlane::objectToWorld(
					nautilus,
					verticesLocal[static_cast<size_t>(localIdx)]);

				Vec3f p;
				if (!projector.tryProjectWorld(camera4D, vWorld, p))
				{
					continue;
				}

				proj[winIdx] = p;
				visible[winIdx] = true;
				dists[winIdx] = hyperDists[static_cast<size_t>(localIdx)];

				minD = std::min(minD, dists[winIdx]);
				maxD = std::max(maxD, dists[winIdx]);
			}
		}

		if (maxD <= minD)
		{
			maxD = minD + 1.f;
		}

		g.blending(true);
		g.blendTrans();

		Mesh lines;
		lines.primitive(Mesh::LINES);

		for (int k = 0; k < ringCountClamped; ++k)
		{
			for (int v = 0; v < vSteps; ++v)
			{
				const size_t curr = static_cast<size_t>(k * vSteps + v);
				const size_t nextV = static_cast<size_t>(k * vSteps + ((v + 1) % vSteps));
				const size_t nextT = static_cast<size_t>((k + 1) * vSteps + v);

				if (!visible[curr])
				{
					continue;
				}

				const float t = clamp01((dists[curr] - minD) / (maxD - minD));
				const al::Color c = nautilusGradientColor(t);

				if (visible[nextV])
				{
					lines.color(c);
					lines.vertex(proj[curr]);
					lines.color(c);
					lines.vertex(proj[nextV]);
				}

				const int actualRingIndex = (startRingClamped + k) % tSteps;
				const bool isPhysicalWrap = (actualRingIndex == tSteps - 1);
				if (!isPhysicalWrap && visible[nextT])
				{
					lines.color(c);
					lines.vertex(proj[curr]);
					lines.color(c);
					lines.vertex(proj[nextT]);
				}
			}
		}

		Mesh points;
		if (nautilus.drawVertexDots() && nautilus.pointStride() > 0)
		{
			points.primitive(Mesh::POINTS);
			g.pointSize(nautilus.pointSize());

			const int pointStride = std::max(1, nautilus.pointStride());
			for (int k = 0; k < ringCountClamped; k += pointStride)
			{
				for (int v = 0; v < vSteps; v += pointStride)
				{
					const size_t idx = static_cast<size_t>(k * vSteps + v);
					if (idx >= windowVerts || !visible[idx])
					{
						continue;
					}

					const float t = clamp01((dists[idx] - minD) / (maxD - minD));
					points.color(nautilusGradientColor(t));
					points.vertex(proj[idx]);
				}
			}
		}

		projector.drawLineMesh(g, lines);

		if (nautilus.drawVertexDots() && !points.vertices().empty())
		{
			projector.drawLineMesh(g, points);
		}

		g.blending(false);
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
		case 'd':
		case 'D':
			camera4D.addMoveSpeedLocal(0, kMoveSpeed);
			return true;
		case 'a':
		case 'A':
			camera4D.addMoveSpeedLocal(0, -kMoveSpeed);
			return true;
		case 'e':
		case 'E':
			camera4D.addMoveSpeedLocal(1, kMoveSpeed);
			return true;
		case 'c':
		case 'C':
			camera4D.addMoveSpeedLocal(1, -kMoveSpeed);
			return true;
		case 'w':
		case 'W':
			camera4D.addMoveSpeedLocal(2, kMoveSpeed);
			return true;
		case 'x':
		case 'X':
			camera4D.addMoveSpeedLocal(2, -kMoveSpeed);
			return true;
		case 'r':
		case 'R':
			camera4D.addMoveSpeedLocal(3, kMoveSpeed);
			return true;
		case 'v':
		case 'V':
			camera4D.addMoveSpeedLocal(3, -kMoveSpeed);
			return true;
		case 'z':
		case 'Z':
			resetNavigationInput();
			camera4D.addRotateSpeedLocal(0, rotSpeedRad);
			return true;
		case 'q':
		case 'Q':
			resetNavigationInput();
			camera4D.addRotateSpeedLocal(0, -rotSpeedRad);
			return true;
		case '1':
			resetNavigationInput();
			camera4D.addRotateSpeedLocal(2, rotSpeedRad);
			return true;
		case '2':
			resetNavigationInput();
			camera4D.addRotateSpeedLocal(2, -rotSpeedRad);
			return true;
		case '3':
			resetNavigationInput();
			camera4D.addRotateSpeedLocal(4, rotSpeedRad);
			return true;
		case '4':
			resetNavigationInput();
			camera4D.addRotateSpeedLocal(4, -rotSpeedRad);
			return true;
		case '5':
			resetNavigationInput();
			camera4D.addRotateSpeedLocal(5, rotSpeedRad);
			return true;
		case '6':
			resetNavigationInput();
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

		switch (key)
		{
		case 'd':
		case 'D':
			camera4D.addMoveSpeedLocal(0, -kMoveSpeed);
			return true;
		case 'a':
		case 'A':
			camera4D.addMoveSpeedLocal(0, kMoveSpeed);
			return true;
		case 'e':
		case 'E':
			camera4D.addMoveSpeedLocal(1, -kMoveSpeed);
			return true;
		case 'c':
		case 'C':
			camera4D.addMoveSpeedLocal(1, kMoveSpeed);
			return true;
		case 'w':
		case 'W':
			camera4D.addMoveSpeedLocal(2, -kMoveSpeed);
			return true;
		case 'x':
		case 'X':
			camera4D.addMoveSpeedLocal(2, kMoveSpeed);
			return true;
		case 'r':
		case 'R':
			camera4D.addMoveSpeedLocal(3, -kMoveSpeed);
			return true;
		case 'v':
		case 'V':
			camera4D.addMoveSpeedLocal(3, kMoveSpeed);
			return true;
		default:
			break;
		}

		return false;
	}

	void onExit() override
	{
#ifdef _WIN32
		spoutSender.ReleaseSender();
#endif
		imguiShutdown();
	}
};

int main()
{
	EoYSspout app;
	app.fps(60);
	app.start();
	return 0;
}
