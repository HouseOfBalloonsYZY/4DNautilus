#pragma once

#include "Nav4D.hpp"

using namespace al;

// A placeable, movable 4D object. Same motion API as before the split:
// Pose4D placement plus Nav4D speeds, step, and halt.
// The viewer is Camera4D, which adds home and toLocal / toWorld.
class Object4D : public Nav4D
{
public:
	Object4D() = default;
	virtual ~Object4D() = default;
};
