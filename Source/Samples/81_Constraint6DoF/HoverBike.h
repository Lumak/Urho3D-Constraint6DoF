//
// Copyright (c) 2008-2019 the Urho3D project.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

#pragma once
#include <Urho3D/Scene/LogicComponent.h>
#include <Urho3D/Input/Controls.h>

#include <Urho3D/Physics/RigidBody.h>
#include <Bullet/BulletDynamics/Vehicle/btRaycastVehicle.h>

namespace Urho3D
{
class Node;
class Scene;
class PhysicsWorld;
}
using namespace Urho3D;

//=============================================================================
//=============================================================================
const int CTRL_FORWARD = 1;
const int CTRL_BACK = 2;
const int CTRL_LEFT = 4;
const int CTRL_RIGHT = 8;
const int CTRL_SPACE = 16;

//=============================================================================
//=============================================================================
class HoverBike : public LogicComponent
{
    URHO3D_OBJECT(HoverBike, LogicComponent);
    
public:
    HoverBike(Context* context);
    virtual ~HoverBike();

    static void RegisterObject(Context* context);
    virtual void OnSetAttribute(const AttributeInfo& attr, const Variant& src);

    virtual void DelayedStart();
    virtual void FixedUpdate(float timeStep);

    bool Create();

    /// Movement controls.
    Controls controls_;

protected:
    void CreateRaycastVehicle();
    void AddWheel(const String &hubNodeName, bool isFrontWheel);
    void GetContraintNode();
    void UpdateConstraint(float engineForce);

protected:
    float engineForce_;
    float maxSpeed_;
    float minBrakeForce_;
    float maxBrakeForce_;
    float minBrakeSpeed_;
    float wheelRadius_;
    float wheelFriction_;

    float steeringIncrement_;
    float steeringClamp_;
    float maxRollAngle_;
    float maxSpeedToMaxRollAngle_;

    float suspensionRestLength_;
    float suspensionStiffness_;
    float suspensionRelaxation_;
    float suspensionCompression_;
    float rollInfluence_;
    String constraintName_;
    float softPitchLimit_;

    WeakPtr<RigidBody>                  rigidBody_;
    WeakPtr<PhysicsWorld>               physicsWorld_;
    WeakPtr<Node>                       nodeConstraint6DoF_;
    btRaycastVehicle::btVehicleTuning   vehicleTuning_;
    btVehicleRaycaster                  *vehicleRaycaster_;
    btRaycastVehicle                    *raycastVehicle_;

    float currentSteering_;

};





