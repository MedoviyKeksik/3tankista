#include "game_state_system.h"
#include "components//turn_component.h"
#include "components/adapter_player_id_component.h"
#include "components/name_component.h"
#include "components/ttc_factories/ttc_component_factory.h"
#include "entities/map/map.h"
#include "entities/player.h"
#include "entities/tank.h"
#include "entities/world.h"
#include "systems/adapter_system.h"
#include <algorithm>

GameStateSystem::GameStateSystem()
{
    RegisterEventCallbacks();
}

GameStateSystem::~GameStateSystem()
{
    UnregisterEventCallbacks();
}

void GameStateSystem::OnGameStateResponseEvent(const GameStateResponseEvent* event)
{
    if (event->gameState.players.size() < event->gameState.numberPlayers)
    {
        // TODO: Check if this crutch works...
        ecs::ecsEngine->SendEvent<GameStateRequestEvent>();
        return;
    }
    auto entityManager    = ecs::ecsEngine->GetEntityManager();
    auto componentManager = ecs::ecsEngine->GetComponentManager();
    auto world            = ecs::ecsEngine->GetEntityManager()->GetEntity(
        ecs::ecsEngine->GetComponentManager()->begin<TurnComponent>()->GetOwner());

    auto turnComponent = world->GetComponent<TurnComponent>();
    turnComponent->SetNumOfTurns(event->gameState.numberTurns);
    turnComponent->SetCurrentTurn(event->gameState.currentTurn);
    turnComponent->SetPlayersNumber(event->gameState.numberPlayers);

    auto adapterPlayerId       = world->GetComponent<AdapterPlayerIdComponent>();
    auto adapterVehicleId      = world->GetComponent<AdapterVehicleIdComponent>();
    auto attackMatrixComponent = componentManager->begin<AttackMatrixComponent>().operator->();
    auto attackMatrix          = attackMatrixComponent->GetAttackMatrix();
    // Create players
    for (auto& now : event->gameState.players)
    {
        auto entity = entityManager->CreateEntity<Player>(now.idx, now.name);
        std::cout << now.idx << " " << entity << "\n";
        adapterPlayerId->Add(now.idx, entity);
        attackMatrix.insert({ entity, std::set<uint64_t>{} });
        entityManager->GetEntity(entity)->GetComponent<PlayerIdComponent>()->SetPlayerId(entity);
    }
    attackMatrixComponent->SetAttackMatrix(attackMatrix);
    for (auto& [key, value] : attackMatrixComponent->GetAttackMatrix())
    {
        std::cout << key << " {";
        for (auto& id : value)
        {
            std::cout << id << ", ";
        }
        std::cout << "}\n";
    }
    // Create observers
    for (auto& now : event->gameState.observers)
    {
        auto entity = entityManager->CreateEntity<Player>(now.idx, now.name);
        adapterPlayerId->Add(now.idx, entity);
        entityManager->GetEntity(entity)->GetComponent<PlayerIdComponent>()->SetPlayerId(entity);
    }

    std::map<GameObjectId, Vector3i> playerPosition;

    TtcComponentFactory factory;
    // Create tanks
    for (auto& tank : event->gameState.vehicles)
    {
        auto entity = entityManager->CreateEntity<Tank>(factory, tank.second.vehicleType);
        componentManager->GetComponent<PlayerIdComponent>(entity)->SetPlayerId(
            adapterPlayerId->Get(tank.second.playerId));
        componentManager->GetComponent<VehicleIdComponent>(entity)->SetVehicleId(entity);
        componentManager->GetComponent<SpawnPositionComponent>(entity)->SetSpawnPosition(tank.second.spawnPosition);
        componentManager->GetComponent<PositionComponent>(entity)->SetPosition(tank.second.position);
        componentManager->GetComponent<CapturePointsComponent>(entity)->SetCapturePoints(tank.second.capturePoints);
        adapterVehicleId->Add(tank.first, entity);
        // Determine playerPosition;
        if (playerPosition.find(tank.second.playerId) == playerPosition.end())
        {
            playerPosition[tank.second.playerId] = tank.second.spawnPosition;
        }
    }

    auto less = [](Vector2i lhs, Vector2i rhs)
    {
        if (lhs.x() >= 0 && rhs.x() < 0)
            return true;
        if (lhs.x() < 0 && rhs.x() >= 0)
            return false;
        if (lhs.x() == 0 && rhs.x() == 0)
        {
            if (lhs.y() >= 0 || rhs.y() >= 0)
                return lhs.y() > rhs.y();
            return rhs.y() > lhs.y();
        }
        int det = lhs.x() * rhs.y() - rhs.x() * lhs.y();
        if (det < 0)
            return true;
        if (det > 0)
            return false;
        int d1 = lhs.x() * lhs.x() + lhs.y() * lhs.y();
        int d2 = rhs.x() * rhs.x() + rhs.y() * rhs.y();
        return d1 > d2;
    };
    std::vector<std::pair<uint64_t, Vector2i>> playerHexPos;
    for (auto& now : playerPosition)
    {
        playerHexPos.push_back({ now.first, { now.second.x(), now.second.z() } });
    }
    std::sort(playerHexPos.begin(),
              playerHexPos.end(),
              [&](std::pair<uint64_t, Vector2i>& lhs, std::pair<uint64_t, Vector2i>& rhs)
              { return less(lhs.second, rhs.second); });
    int index = 0;
    while (index < playerHexPos.size())
    {
        if (event->gameState.currentPlayerIndex == playerHexPos[index].first)
            break;
        index++;
    }
    auto turn            = event->gameState.currentTurn;
    auto playersNum      = event->gameState.numberPlayers;
    auto mainPlayerIndex = componentManager->begin<MainPlayerComponent>()->GetMainPlayerId();
    componentManager->begin<MainPlayerComponent>()->SetMainPlayerId(adapterPlayerId->Get(mainPlayerIndex));
    for (int i = index; i > -1; --i)
    {
        componentManager->GetComponent<OrderComponent>(adapterPlayerId->Get(playerHexPos[i].first))
            ->SetOrder(turn % playersNum);
        turn++;
    }

    //    for (int i = index; i < playerHexPos.size(); i++)
    //    {
    //        componentManager->GetComponent<OrderComponent>(adapterPlayerId->Get(playerHexPos[i].first))
    //            ->SetOrder(turn % playersNum);
    //        turn++;
    //    }
    //    for (int i = 0; i < index; i++)
    //    {
    //        componentManager->GetComponent<OrderComponent>(adapterPlayerId->Get(playerHexPos[i].first))
    //            ->SetOrder(turn % playersNum);
    //        turn++;
    //    }
    auto mainPlayerId = componentManager->begin<MainPlayerComponent>()->GetMainPlayerId();
    for (auto it = componentManager->begin<VehicleIdComponent>(); componentManager->end<VehicleIdComponent>() != it;
         ++it)
    {
        auto tank = (Tank*)entityManager->GetEntity(it->GetVehicleId());
        if (tank->GetComponent<PlayerIdComponent>()->GetPlayerId() == mainPlayerId)
        {
            GameplaySystem::SetHexMapComponentCell(world->GetComponent<HexMapComponent>(),
                                                   tank->GetComponent<PositionComponent>()->GetPosition(),
                                                   CellState::FRIEND);
        }
        else
        {
            GameplaySystem::SetHexMapComponentCell(world->GetComponent<HexMapComponent>(),
                                                   tank->GetComponent<PositionComponent>()->GetPosition(),
                                                   CellState::ENEMY);
        }
    }
}

void GameStateSystem::OnWorldCreateEvent(const WorldCreateEvent* event)
{
    ecs::ecsEngine->GetEntityManager()->CreateEntity<World>();
}

void GameStateSystem::RegisterEventCallbacks()
{
    RegisterEventCallback(&GameStateSystem::OnGameStateResponseEvent);
    RegisterEventCallback(&GameStateSystem::OnWorldCreateEvent);
}

void GameStateSystem::UnregisterEventCallbacks()
{
    UnregisterEventCallback(&GameStateSystem::OnGameStateResponseEvent);
    UnregisterEventCallback(&GameStateSystem::OnWorldCreateEvent);
}
