
#pragma once

#include "raylib.h"

#include "../monde/Monde.h"
#include "../robot/Robot.h"
#include "../cible/Cible.h"
#include "../navigation/NavigationController.h"

void DrawWorld3D(
    const World& world,
    const Robot& robot,
    const Target& target,
    const Camera3D& camera,
    const NavigationController& navigationController
);
