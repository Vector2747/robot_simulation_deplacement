
#pragma once

#include "../core/Config.h"
#include "../monde/Monde.h"
#include "../robot/Robot.h"
#include "../cible/Cible.h"
#include "../input/ManualControlleur.h"
#include "../navigation/NavigationController.h"
#include "../monde/EvilAhrEvent.h"

void DrawDashboard(
    World& world,
    Robot& robot,
    const Target& target,
    const ManualController& manualController,
    NavigationController& navigationController,
    EvilAhrEvent& evilAhrEvent,
    SimulationConfig& config
);
