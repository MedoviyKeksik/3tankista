#pragma once

#include "components/obstacle_id_component.h"
#include "components/position_component.h"
#include "game/game_object.h"
#include <memory>

class Obstacle : public GameObject<Obstacle>
{
public:
    Obstacle(const ecs::EntityId& entityId, ecs::ComponentManager* componentManager);
    ~Obstacle() override = default;

private:
    ObstacleIdComponent* obstacleId;
    PositionComponent*   position;
};