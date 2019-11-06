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

#include <Urho3D/Core/Context.h>
#include <Urho3D/Core/CoreEvents.h>
#include <Urho3D/Graphics/DebugRenderer.h>
#include <Urho3D/Physics/CollisionShape.h>
#include <Urho3D/Physics/PhysicsWorld.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/Physics/Constraint6DoF.h>
#include <Urho3D/Physics/PhysicsUtils.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Scene/Node.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/IO/Log.h>

#include <cassert>

#include "HoverBike.h"

#include <Urho3D/DebugNew.h>
//=============================================================================
//=============================================================================
const Vector3 wheelDirectionCS0(0,-1,0);
const Vector3 wheelAxleCS(-1,0,0);
const int rightIndex = 0;
const int upIndex = 1;
const int forwardIndex = 2;

//=============================================================================
//=============================================================================
HoverBike::HoverBike(Context* context)
    : LogicComponent(context)
    , raycastVehicle_(NULL)
    , vehicleRaycaster_(NULL)
    , engineForce_(10.0f)
    , maxSpeed_(150.0f)
    , minBrakeForce_(2.0f)
    , maxBrakeForce_(300.0f)
    , wheelRadius_(0.9f)
    , wheelFriction_(0.9f)

    , steeringIncrement_(0.02f)
    , steeringClamp_(0.4f)
    , maxRollAngle_(30.0f)
    , maxSpeedToMaxRollAngle_(120.0f)

    , suspensionRestLength_(0.7f)
    , suspensionStiffness_(25.0f)
    , suspensionRelaxation_(7.0f)
    , suspensionCompression_(10.0f)
    , rollInfluence_(0.3f)

    , currentSteering_(0.0f)
    , softPitchLimit_(40.0f)
{
    SetUpdateEventMask(0);
}

HoverBike::~HoverBike()
{
    if (vehicleRaycaster_)
    {
        delete vehicleRaycaster_;
        vehicleRaycaster_ = NULL;
    }
    if (raycastVehicle_)
    {
        if (physicsWorld_)
        {
            btDynamicsWorld *dynamicWorld = (btDynamicsWorld*)physicsWorld_->GetWorld();
            dynamicWorld->removeVehicle(raycastVehicle_);
        }

        delete raycastVehicle_;
        raycastVehicle_ = NULL;
    }
}

void HoverBike::RegisterObject(Context* context)
{
    context->RegisterFactory<HoverBike>();

    URHO3D_ATTRIBUTE("Engine Force", float, engineForce_, 10.0f, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Max Speed", float, maxSpeed_, 150.0f, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Min Brake Force", float, minBrakeForce_, 2.0f, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Max Brake Force", float, maxBrakeForce_, 300.0f, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Wheel Radius", float, wheelRadius_, 0.9f, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Wheel Friction", float, wheelFriction_, 0.9f, AM_DEFAULT);

    URHO3D_ATTRIBUTE("Steering Increment", float, steeringIncrement_, 0.03f, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Steering Clamp", float, steeringClamp_, 0.5f, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Max Roll Angle", float, maxRollAngle_, 30.0f, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Max SpeedToRoll Angle", float, maxSpeedToMaxRollAngle_, 120.0f, AM_DEFAULT);

    URHO3D_ATTRIBUTE("Suspension Rest Length", float, suspensionRestLength_, 0.7f, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Suspension Stiffness", float, suspensionStiffness_, 20.0f, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Suspension Relaxation", float, suspensionRelaxation_, 4.0f, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Suspension Compression", float, suspensionCompression_, 5.0f, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Roll Influence", float, rollInfluence_, 0.3f, AM_DEFAULT);

    URHO3D_ATTRIBUTE("Constraint Name", String, constraintName_, String::EMPTY, AM_DEFAULT);
    URHO3D_ATTRIBUTE("SoftPitch Limit", float, softPitchLimit_, 40.0f, AM_DEFAULT);
}

void HoverBike::OnSetAttribute(const AttributeInfo& attr, const Variant& src)
{
    Serializable::OnSetAttribute(attr, src);
}

void HoverBike::DelayedStart()
{
    // create raycast vehicle
    if (raycastVehicle_ == NULL)
    {
        Create();
    }
}

bool HoverBike::Create()
{
    rigidBody_ = GetComponent<RigidBody>();

    if (rigidBody_ == NULL)
    {
        return false;
    }

    CreateRaycastVehicle();
    GetContraintNode();

    // activate
    SetUpdateEventMask(USE_FIXEDUPDATE);
    return true;
}

void HoverBike::CreateRaycastVehicle()
{
    if (raycastVehicle_)
    {
        return;
    }

    // create raycast vehicle
    physicsWorld_ = rigidBody_->GetPhysicsWorld();
    btDynamicsWorld *pbtDynWorld = (btDynamicsWorld*)physicsWorld_->GetWorld();
    vehicleRaycaster_ = new btDefaultVehicleRaycaster(pbtDynWorld);
    raycastVehicle_ = new btRaycastVehicle(vehicleTuning_, rigidBody_->GetBody(), vehicleRaycaster_);

    pbtDynWorld->addVehicle(raycastVehicle_);
    raycastVehicle_->setCoordinateSystem(rightIndex, upIndex, forwardIndex);

    // add wheels
    AddWheel("frontHub", true);
    AddWheel("rearHub", false);

    // config suspension
    for (int i = 0; i < raycastVehicle_->getNumWheels(); ++i)
    {
        btWheelInfo& wheel = raycastVehicle_->getWheelInfo(i);
        wheel.m_suspensionStiffness = suspensionStiffness_ ;
        wheel.m_wheelsDampingRelaxation = suspensionRelaxation_;
        wheel.m_wheelsDampingCompression = suspensionCompression_;
        wheel.m_frictionSlip = wheelFriction_;
        wheel.m_rollInfluence = rollInfluence_;
    }
}

void HoverBike::AddWheel(const String &hubNodeName, bool isFrontWheel)
{
    Node *hub = node_->GetChild(hubNodeName, true);
    assert(hub != NULL && "Hub missing.");

    Vector3 pointCS0 = hub->GetPosition();
    raycastVehicle_->addWheel(ToBtVector3(pointCS0), ToBtVector3(wheelDirectionCS0), ToBtVector3(wheelAxleCS),
                              suspensionRestLength_, wheelRadius_, vehicleTuning_, isFrontWheel);
}

void HoverBike::GetContraintNode()
{
    nodeConstraint6DoF_ = GetScene()->GetChild(constraintName_, true);
}

void HoverBike::FixedUpdate(float timeStep)
{
    float newSteering = 0.0f;
    float accelerator = 0.0f;
    bool braking = false;

    // Read controls
    if (controls_.buttons_ & CTRL_LEFT)
        newSteering = -1.0f;
    if (controls_.buttons_ & CTRL_RIGHT)
        newSteering = 1.0f;
    if (controls_.buttons_ & CTRL_FORWARD)
        accelerator = 1.0f;
    if (controls_.buttons_ & CTRL_BACK)
        accelerator = -0.4f;
    if (controls_.buttons_ & CTRL_SPACE)
    {
        braking = true;
        accelerator = 0.0f;
    }

    if (newSteering != 0.0f || accelerator != 0.0f)
    {
        rigidBody_->Activate();
    }

    if (newSteering != 0.0f)
    {
        currentSteering_ += steeringIncrement_ * newSteering;
    }
    else
    {
        currentSteering_ *= 0.7f;
    }
    currentSteering_ = Clamp(currentSteering_, -steeringClamp_, steeringClamp_);
    raycastVehicle_->setSteeringValue(currentSteering_, 0);
    Quaternion rot = rigidBody_->GetRotation();
    float engineForce = (Abs(accelerator)>0.0f && !braking)?engineForce_ * accelerator:0.0f;

    for (int i = 0; i < raycastVehicle_->getNumWheels(); ++i)
    {
        raycastVehicle_->applyEngineForce(engineForce, i);

        if (i > 0)
        {
            raycastVehicle_->setBrake(braking?maxBrakeForce_:minBrakeForce_, i);
        }
    }

    UpdateConstraint(engineForce);
}

void HoverBike::UpdateConstraint(float engineForce)
{
    if (nodeConstraint6DoF_)
    {
        nodeConstraint6DoF_->SetWorldPosition(node_->GetWorldPosition());

        // update rotation
        Vector3 dir = node_->GetWorldDirection();
        dir.y_ = 0.0f;
        if (Abs(currentSteering_) > M_EPSILON)
        {
            Quaternion rot = nodeConstraint6DoF_->GetWorldRotation();
            Vector3 dofdir = rot * Vector3::RIGHT * currentSteering_;
            dofdir.y_ = 0.0f;
            if (engineForce < -M_EPSILON)
            {
                dofdir *= -1.0f;
            }
            dir += dofdir;
        }
        dir.Normalized();
        nodeConstraint6DoF_->SetWorldDirection(dir);

        // update roll
        float velLen = rigidBody_->GetLinearVelocity().Length();
        velLen = Clamp(velLen, 0.0f, maxSpeedToMaxRollAngle_);
        float ilerp = InverseLerp(0.0f, maxSpeedToMaxRollAngle_, velLen);
        float roll = ilerp * maxRollAngle_ * Abs(currentSteering_)/steeringClamp_;

        if (roll > 0.0f)
        {
            Vector3 rgt = node_->GetWorldRight();
            rgt.y_ = 0.0f;
            rgt.Normalized();
            Quaternion nrot;
            nrot.FromAxes(rgt, Vector3::UP, dir);
            Quaternion rollRot = Quaternion(-Sign(currentSteering_) * roll, Vector3(0,0,1));
            nodeConstraint6DoF_->SetWorldRotation(nrot * rollRot);
        }

        // soft pitch limit
        if (softPitchLimit_ > M_EPSILON)
        {
            Quaternion brot = rigidBody_->GetRotation();
            Constraint6DoF *constraint6DoF = nodeConstraint6DoF_->GetComponent<Constraint6DoF>();
            const Vector3 angLowLimit = constraint6DoF->GetAngularLowerLimit();
            const Vector3 angUppLimit = constraint6DoF->GetAngularUpperLimit();
            Vector3 nangLowLimit(angLowLimit);
            Vector3 nangUppLimit(angUppLimit);

            if (Abs(brot.PitchAngle()) > softPitchLimit_)
            {
                nangLowLimit.x_ = -M_LARGE_EPSILON;
                nangUppLimit.x_ = M_LARGE_EPSILON;
                constraint6DoF->SetAngularLowerLimit(nangLowLimit);
                constraint6DoF->SetAngularUpperLimit(nangUppLimit);
            }
            else if (Abs(brot.PitchAngle()) < softPitchLimit_ * 0.2f && nangLowLimit.x_ < nangUppLimit.x_)
            {
                nangLowLimit.x_ = 1.0f;
                nangUppLimit.x_ = 0.0f;
                constraint6DoF->SetAngularLowerLimit(nangLowLimit);
                constraint6DoF->SetAngularUpperLimit(nangUppLimit);
            }
        }
    }
}













