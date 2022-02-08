#include "gameplay.hpp"

GameAlgorithm::GameAlgorithm(std::shared_ptr<GameArea> gameArea,
                             std::shared_ptr<Map>      map)
    : gameArea(std::move(gameArea))
    , gameState(nullptr)
    , map(std::move(map))
    , pathFinder(new PathFinder(this->gameArea))
{
}

bool GameAlgorithm::CheckNeutrality(std::shared_ptr<AbstractTank>& playerTank,
                                    std::shared_ptr<AbstractTank>& enemyTank)
{
    bool neutrality    = false;
    auto attackMatrix  = gameState->GetAttackMatrix();
    int  tankPlayerId  = playerTank->GetPlayerId();
    int  enemyPlayerId = enemyTank->GetPlayerId();
    int  thirdPlayerId = -1;
    for (const auto& [key, value] : attackMatrix)
    {
        if (key != enemyPlayerId && key != tankPlayerId)
        {
            thirdPlayerId = key;
            break;
        }
    }
    if (std::find(attackMatrix[enemyPlayerId].begin(),
                  attackMatrix[enemyPlayerId].end(),
                  tankPlayerId) != attackMatrix[enemyPlayerId].end() ||
        std::find(attackMatrix[thirdPlayerId].begin(),
                  attackMatrix[thirdPlayerId].end(),
                  enemyPlayerId) == attackMatrix[thirdPlayerId].end())
    {
        neutrality = true;
    }
    return neutrality;
}

bool GameAlgorithm::IsCorrectShootPosition(
    const std::shared_ptr<AbstractTank>& tank, const Vector3i& position)
{
    bool result = false;
    if (tank->GetTankType() == TankType::AT_SPG)
    {
        if (pathFinder->GetDistance(position) != NOPATH &&
            pathFinder->GetDistance(position) ==
                GameArea::GetDistance(tank->GetPosition(), position))
        {
            result = true;
        }
    }
    else
    {
        result = gameArea->GetCell(position) != CellState::OBSTACLE;
    }
    return result;
}

void GameAlgorithm::Play()
{
    gameArea->ClearMap();
    std::vector<std::shared_ptr<AbstractTank>>              currentPlayerTanks;
    std::vector<std::vector<std::shared_ptr<AbstractTank>>> enemies;
    for (auto& now : gameState->GetVehicles())
    {
        if (now.first == gameState->GetCurrentPlayerIdx())
        {
            currentPlayerTanks = now.second;
            for (auto& vehicle : now.second)
            {
                gameArea->SetCell(vehicle->GetPosition(), CellState::FRIEND);
            }
        }
        else
        {
            enemies.push_back(now.second);
            for (auto& vehicle : now.second)
            {
                gameArea->SetCell(vehicle->GetPosition(), CellState::ENEMY);
            }
        }
    }
    for (auto& cell : map->GetContent().GetObstacle())
    {
        gameArea->SetCell(cell, CellState::OBSTACLE);
    }
    std::sort(currentPlayerTanks.begin(),
              currentPlayerTanks.end(),
              [](auto& lhs, auto& rhs) { return *lhs < *rhs; });
    for (auto& tank : currentPlayerTanks)
    {
        // Can attack someone?
        pathFinder->SetStartPoint(tank->GetPosition());
        std::shared_ptr<AbstractTank> target = nullptr;
        for (auto& enemyArray : enemies)
        {
            for (auto& potentialTarget : enemyArray)
            {
                /* check neutrality rule && gameState->GetAttackMatrix()*/
                if (tank->CanShoot(potentialTarget->GetPosition()) &&
                    IsCorrectShootPosition(tank,
                                           potentialTarget->GetPosition()) &&
                    CheckNeutrality(tank, potentialTarget))
                {
                    if (target == nullptr ||
                        target->GetHealth() > potentialTarget->GetHealth())
                    {
                        target = potentialTarget;
                    }
                }
            }
        }
        if (target != nullptr)
        {
            tank->Shoot(target->GetPosition());
        }
        else
        {
            // Is on the base?
            bool isOnTheBase = false;
            for (auto& basePosition : map->GetContent().GetBase())
            {
                isOnTheBase |= basePosition == tank->GetPosition();
            }
            if (isOnTheBase)
                continue; // STAY
            // Move to the nearest base
            Vector3i nearestBasePos = map->GetContent().GetBase().front();
            for (auto& basePosition : map->GetContent().GetBase())
            {
                if (pathFinder->GetDistance(nearestBasePos) == NOPATH ||
                    (pathFinder->GetDistance(nearestBasePos) >
                         pathFinder->GetDistance(basePosition) &&
                     pathFinder->GetDistance(basePosition) != NOPATH))
                {
                    nearestBasePos = basePosition;
                }
            }
            if (pathFinder->GetDistance(nearestBasePos) == NOPATH)
                continue; // STAY
            auto path = pathFinder->GetShortestPath(nearestBasePos);
            if (path.empty())
                continue; // STAY
            gameArea->SetCell(tank->GetPosition(), CellState::EMPTY);
            gameArea->SetCell(
                path[std::min((int)path.size(), tank->GetSpeed()) - 1],
                CellState::FRIEND);
            tank->Move(path[std::min((int)path.size(), tank->GetSpeed()) - 1]);
        }
    }
}
