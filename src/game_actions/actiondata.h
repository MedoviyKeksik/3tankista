#ifndef ACTIONDATA_H
#define ACTIONDATA_H

#include "matrix.hpp"

class ActionData
{
public:
    ActionData();
    ActionData(const int vehicleId, const Vector3i& target);
    virtual ~ActionData();

public:
    void SetVehicleId(const int vehicleId) { this->vehicleId = vehicleId; }
    int& GetVehicleId() { return this->vehicleId; }
    const int& GetVehicleId() const { return this->vehicleId; }

    void            SetTarget(const Vector3i& target) { this->target = target; }
    Vector3i&       GetTarget() { return this->target; }
    const Vector3i& GetTarget() const { return this->target; }

private:
    int      vehicleId;
    Vector3i target;
};

void to_json(nlohmann::json& json, const ActionData& actionData);

void from_json(const nlohmann::json& json, ActionData& actionData);

#endif // ACTIONDATA_H