#include "spgtank.h"

SpgTank::SpgTank(int vehicleId)
    : AbstractTank(vehicleId, TankType::SPG)
{
    SetHealth(1);
    SetMaxHealth(1);
    SetSpeed(1);
    SetDamage(1);
    SetDestructionPoints(1);
}

SpgTank::~SpgTank() {}

bool SpgTank::CanShoot(const Vector3i& point) const
{
    return GameArea::GetDistance(point, this->GetPosition()) == 3;
}

bool SpgTank::CanMove(const Vector3i& point) const
{
    return GameArea::GetDistance(point, this->GetPosition()) <=
           this->GetSpeed();
}