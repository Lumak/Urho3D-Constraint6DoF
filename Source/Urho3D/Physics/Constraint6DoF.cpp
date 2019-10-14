//
// Copyright (c) 2008-2017 the Urho3D project.
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

#include "../Precompiled.h"

#include "../Core/Context.h"
#include "../Core/Profiler.h"
#include "../Graphics/DebugRenderer.h"
#include "../IO/Log.h"
#include "../Physics/Constraint6DoF.h"
#include "../Physics/PhysicsUtils.h"
#include "../Physics/PhysicsWorld.h"
#include "../Physics/RigidBody.h"
#include "../Scene/Scene.h"

#include <Bullet/BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h>
#include <Bullet/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>

namespace Urho3D
{

extern const char* PHYSICS_CATEGORY;

static const char* typeNames[] =
{
    "Generic6DoFSpring2",
    0
};
const Vector3 DefaultERP(0.2f,0.2f,0.2f);

Constraint6DoF::Constraint6DoF(Context* context)
    : Constraint(context)
    , linearLowerLimit_(Vector3::ONE)
    , linearUpperLimit_(Vector3::ZERO)
    , angularLowerLimit_(Vector3::ONE)
    , angularUpperLimit_(Vector3::ZERO)

    , linearServoEnable_(IntVector3::ZERO)
    , angularServoEnable_(IntVector3::ZERO)

    , linearMotorEnable_(IntVector3::ZERO)
    , angularMotorEnable_(IntVector3::ZERO)
    , linearMotorForce_(Vector3::ZERO)
    , angularMotorForce_(Vector3::ZERO)

    , linearBounceForce_(Vector3::ZERO)
    , angularBounceForce_(Vector3::ZERO)

    , linearSpringEnable_(IntVector3::ZERO)
    , angularSpringEnable_(IntVector3::ZERO)

    , linearStiffness_(Vector3::ZERO)
    , angularStiffness_(Vector3::ZERO)
    , linearStiffnessLimit_(IntVector3::ONE)
    , angularStiffnessLimit_(IntVector3::ONE)

    , linearDamping_(Vector3::ZERO)
    , angularDamping_(Vector3::ZERO)
    , linearDampingLimit_(IntVector3::ONE)
    , angularDampingLimit_(IntVector3::ONE)

    , linearErps_(DefaultERP)
    , angularErps_(DefaultERP)
    , linearCfms_(Vector3::ZERO)
    , angularCfms_(Vector3::ZERO)

    , linearStopErps_(DefaultERP)
    , angularStopErps_(DefaultERP)
    , linearStopCfms_(Vector3::ZERO)
    , angularStopCfms_(Vector3::ZERO)
{
    constraintType_ = CONSTRAINT_GENERIC6DOFSPRING2;
}

Constraint6DoF::~Constraint6DoF()
{
    ReleaseConstraint();

    if (physicsWorld_)
        physicsWorld_->RemoveConstraint(this);
}

void Constraint6DoF::RegisterObject(Context* context)
{
    context->RegisterFactory<Constraint6DoF>(PHYSICS_CATEGORY);

    URHO3D_COPY_BASE_ATTRIBUTES(Constraint);

    // **note** change base Constraint attributes - add AM_NOEDIT mask
    URHO3D_ENUM_ATTRIBUTE("Constraint Type", constraintType_, typeNames, CONSTRAINT_GENERIC6DOFSPRING2, AM_DEFAULT | AM_NOEDIT);
    URHO3D_ACCESSOR_ATTRIBUTE("High Limit", GetHighLimit, SetHighLimit, Vector2, Vector2::ZERO, AM_DEFAULT | AM_NOEDIT);
    URHO3D_ACCESSOR_ATTRIBUTE("Low Limit", GetLowLimit, SetLowLimit, Vector2, Vector2::ZERO, AM_DEFAULT | AM_NOEDIT);
    URHO3D_ACCESSOR_ATTRIBUTE("ERP Parameter", GetERP, SetERP, float, 0.0f, AM_DEFAULT | AM_NOEDIT);
    URHO3D_ACCESSOR_ATTRIBUTE("CFM Parameter", GetCFM, SetCFM, float, 0.0f, AM_DEFAULT | AM_NOEDIT);

    // DoF Spring2 attrib
    URHO3D_ACCESSOR_ATTRIBUTE("Lower Linear Limit", GetLinearLowerLimit, SetLinearLowerLimit, Vector3, Vector3::ONE, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Upper Linear Limit", GetLinearUpperLimit, SetLinearUpperLimit, Vector3, Vector3::ZERO, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Lower Angular Limit", GetAngularLowerLimit, SetAngularLowerLimit, Vector3, Vector3::ONE, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Upper Angular Limit", GetAngularUpperLimit, SetAngularUpperLimit, Vector3, Vector3::ZERO, AM_DEFAULT);

    URHO3D_ACCESSOR_ATTRIBUTE("Enable Linear Servo", GetEnableLinearServo, SetEnableLinearServo, IntVector3, IntVector3::ZERO, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Enable Angular Servo", GetEnableAngularServo, SetEnableAngularServo, IntVector3, IntVector3::ZERO, AM_DEFAULT);

    URHO3D_ACCESSOR_ATTRIBUTE("Enable Linear Motor", GetEnableLinearMotor, SetEnableLinearMotor, IntVector3, IntVector3::ZERO, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Enable Angular Motor", GetEnableAngularMotor, SetEnableAngularMotor, IntVector3, IntVector3::ZERO, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Linear MaxMotor Force", GetLinearMaxMotorForce, SetLinearMaxMotorForce, Vector3, Vector3::ZERO, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Angular MaxMotor Force", GetAngularMaxMotorForce, SetAngularMaxMotorForce, Vector3, Vector3::ZERO, AM_DEFAULT);

    URHO3D_ACCESSOR_ATTRIBUTE("Linear Bounce", GetLinearBounce, SetLinearBounce, Vector3, Vector3::ZERO, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Angular Bounce", GetAngularBounce, SetAngularBounce, Vector3, Vector3::ZERO, AM_DEFAULT);

    URHO3D_ACCESSOR_ATTRIBUTE("Enable Linear Spring", GetEnableLinearSpring, SetEnableLinearSpring, IntVector3, IntVector3::ZERO, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Enable Angular Spring", GetEnableAngularSpring, SetEnableAngularSpring, IntVector3, IntVector3::ZERO, AM_DEFAULT);

    URHO3D_ACCESSOR_ATTRIBUTE("Linear Stiffness", GetLinearStiffness, SetLinearStiffness, Vector3, Vector3::ZERO, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Angular Stiffness", GetAngularStiffness, SetAngularStiffness, Vector3, Vector3::ZERO, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Limit Linear Stiffness", GetLimitLinearStiffness, SetLimitLinearStiffness, IntVector3, IntVector3::ONE, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Limit Angular Stiffness", GetLimitAngularStiffness, SetLimitAngularStiffness, IntVector3, IntVector3::ONE, AM_DEFAULT);

    URHO3D_ACCESSOR_ATTRIBUTE("Linear Damping", GetLinearDamping, SetLinearDamping, Vector3, Vector3::ZERO, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Angular Damping", GetAngularDamping, SetAngularDamping, Vector3, Vector3::ZERO, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Limit Linear Damping", GetLimitLinearDamping, SetLimitLinearDamping, IntVector3, IntVector3::ONE, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Limit Angular Damping", GetLimitAngularDamping, SetLimitAngularDamping, IntVector3, IntVector3::ONE, AM_DEFAULT);

    URHO3D_ACCESSOR_ATTRIBUTE("Linear ERP Parameters", GetLinearERPs, SetLinearERPs, Vector3, DefaultERP, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Angular ERP Parameters", GetAngularERPs, SetAngularERPs, Vector3, DefaultERP, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Linear CFM Parameters", GetLinearCFMs, SetLinearCFMs, Vector3, Vector3::ZERO, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Angular CFM Parameters", GetAngularCFMs, SetAngularCFMs, Vector3, Vector3::ZERO, AM_DEFAULT);

    URHO3D_ACCESSOR_ATTRIBUTE("Linear STOP ERP Parameters", GetLinearStopERPs, SetLinearStopERPs, Vector3, DefaultERP, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Angular STOP ERP Parameters", GetAngularStopERPs, SetAngularStopERPs, Vector3, DefaultERP, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Linear STOP CFM Parameters", GetLinearStopCFMs, SetLinearStopCFMs, Vector3, Vector3::ZERO, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Angular STOP CFM Parameters", GetAngularStopCFMs, SetAngularStopCFMs, Vector3, Vector3::ZERO, AM_DEFAULT);
}

void Constraint6DoF::OnSetAttribute(const AttributeInfo& attr, const Variant& src)
{
    Serializable::OnSetAttribute(attr, src);
    AttributeInfo *pattr = const_cast<AttributeInfo*>(&attr);

    if (!attr.accessor_)
    {
        // Convenience for editing static constraints: if not connected to another body, adjust world position to match local
        // (when deserializing, the proper other body position will be read after own position, so this calculation is safely
        // overridden and does not accumulate constraint error
        if (attr.offset_ == offsetof(Constraint6DoF, position_) && constraint_ && !otherBody_)
        {
            btTransform ownBody = constraint_->getRigidBodyA().getWorldTransform();
            btVector3 worldPos = ownBody * ToBtVector3(position_ * cachedWorldScale_ - ownBody_->GetCenterOfMass());
            otherPosition_ = ToVector3(worldPos);
        }

        // Certain attribute changes require recreation of the constraint
        if (attr.offset_ == offsetof(Constraint6DoF, constraintType_) || attr.offset_ == offsetof(Constraint6DoF, otherBodyNodeID_) ||
            attr.offset_ == offsetof(Constraint6DoF, disableCollision_))
            recreateConstraint_ = true;
        else
            framesDirty_ = true;
    }
}

void Constraint6DoF::ApplyFrames()
{
    if (!constraint_ || !node_ || (otherBody_ && !otherBody_->GetNode()))
        return;

    cachedWorldScale_ = node_->GetWorldScale();

    //lumak
    // setFrames() is not called for CONSTRAINT_GENERIC6DOFSPRING2 to avoid recreating jacobian matrix
}

void Constraint6DoF::CreateConstraint()
{
    URHO3D_PROFILE(CreateConstraint);

    cachedWorldScale_ = node_->GetWorldScale();

    ReleaseConstraint();

    ownBody_ = GetComponent<RigidBody>();
    btRigidBody* ownBody = ownBody_ ? ownBody_->GetBody() : 0;
    btRigidBody* otherBody = otherBody_ ? otherBody_->GetBody() : 0;

    // If no physics world available now mark for retry later
    if (!physicsWorld_ || !ownBody)
    {
        retryCreation_ = true;
        return;
    }

    if (!otherBody)
    {
        otherBody = &btTypedConstraint::getFixedBody();
    }

    Vector3 ownBodyScaledPosition = position_ * cachedWorldScale_ - ownBody_->GetCenterOfMass();
    Vector3 otherBodyScaledPosition = otherBody_ ? otherPosition_ * otherBody_->GetNode()->GetWorldScale() -
                                                   otherBody_->GetCenterOfMass() : otherPosition_;

    btTransform ownFrame(ToBtQuaternion(rotation_), ToBtVector3(ownBodyScaledPosition));
    btTransform otherFrame(ToBtQuaternion(otherRotation_), ToBtVector3(otherBodyScaledPosition));
    btGeneric6DofSpring2Constraint *gen6dofConstraint = new btGeneric6DofSpring2Constraint(*ownBody, *otherBody, ownFrame, otherFrame);
    gen6dofConstraint->setDbgDrawSize(0.5f);
    constraint_ = gen6dofConstraint;

    if (constraint_)
    {
        constraint_->setUserConstraintPtr(this);
        constraint_->setEnabled(IsEnabledEffective());
        ownBody_->AddConstraint(this);
        if (otherBody_)
            otherBody_->AddConstraint(this);

        ApplyLimits();

        physicsWorld_->GetWorld()->addConstraint(constraint_.Get(), disableCollision_);
    }

    recreateConstraint_ = false;
    framesDirty_ = false;
    retryCreation_ = false;
}

void Constraint6DoF::ApplyLimits()
{
    if (!constraint_)
        return;

    // push settings to Bullet
    Constraint::ApplyLimits();
    SetLimits();
    SetServo();
    SetEnableMotor();
    SetMaxMotorForce();
    SetBounce();
    SetEnableSpring();
    SetStiffness();
    SetDamping();
    SetERPs();
    SetCFMs();
    SetStopERPs();
    SetStopCFMs();
}

void Constraint6DoF::SetLinearLowerLimit(const Vector3& linearLower)
{
    if (linearLower != linearLowerLimit_)
    {
        linearLowerLimit_ = linearLower;

        ApplyLimits();
        MarkNetworkUpdate();
    }
}

const Vector3& Constraint6DoF::GetLinearLowerLimit() const
{
    return linearLowerLimit_;
}

void Constraint6DoF::SetLinearUpperLimit(const Vector3& linearUpper)
{
    if (linearUpper != linearUpperLimit_)
    {
        linearUpperLimit_ = linearUpper;

        ApplyLimits();
        MarkNetworkUpdate();
    }
}

const Vector3& Constraint6DoF::GetLinearUpperLimit() const
{
    return linearUpperLimit_;
}

void Constraint6DoF::SetAngularLowerLimit(const Vector3& angularLower)
{
    if (angularLower != angularLowerLimit_)
    {
        angularLowerLimit_ = angularLower;

        ApplyLimits();
        MarkNetworkUpdate();
    }
}

const Vector3& Constraint6DoF::GetAngularLowerLimit() const
{
    return angularLowerLimit_;
}

void Constraint6DoF::SetAngularUpperLimit(const Vector3& angularUpper)
{
    if (angularUpper != angularUpperLimit_)
    {
        angularUpperLimit_ = angularUpper;

        ApplyLimits();
        MarkNetworkUpdate();
    }
}

const Vector3& Constraint6DoF::GetAngularUpperLimit() const
{
    return angularUpperLimit_;
}

bool Constraint6DoF::IsLimited(unsigned idx) const
{
    btGeneric6DofSpring2Constraint *gen6dofConstraint = static_cast<btGeneric6DofSpring2Constraint*>(constraint_.Get());
    return gen6dofConstraint->isLimited( (int)idx );
}

void Constraint6DoF::SetLimits()
{
    btGeneric6DofSpring2Constraint *gen6dofConstraint = static_cast<btGeneric6DofSpring2Constraint*>(constraint_.Get());
    gen6dofConstraint->setLinearLowerLimit(ToBtVector3(linearLowerLimit_));
    gen6dofConstraint->setLinearUpperLimit(ToBtVector3(linearUpperLimit_));
    gen6dofConstraint->setAngularLowerLimit(ToBtVector3(angularLowerLimit_ * M_DEGTORAD));
    gen6dofConstraint->setAngularUpperLimit(ToBtVector3(angularUpperLimit_ * M_DEGTORAD));
 }

void Constraint6DoF::SetEnableLinearServo(const IntVector3& enable)
{
    if (enable != linearServoEnable_)
    {
        linearServoEnable_ = enable;

        ApplyLimits();
        MarkNetworkUpdate();
    }
}

const IntVector3& Constraint6DoF::GetEnableLinearServo() const
{
    return linearServoEnable_;
}

void Constraint6DoF::SetEnableAngularServo(const IntVector3& enable)
{
    if (enable != linearServoEnable_)
    {
        angularServoEnable_ = enable;

        ApplyLimits();
        MarkNetworkUpdate();
    }
}

const IntVector3& Constraint6DoF::GetEnableAngularServo() const
{
    return angularServoEnable_;
}

void Constraint6DoF::SetEnableServo(unsigned idx, bool enable)
{
    if ( idx <= D6_LINEAR_Z )
    {
        if ( enable != !EqualZero( linearServoEnable_.Data()[idx] ) )
        {
            (&linearServoEnable_.x_)[idx] = enable?1:0;
            ApplyLimits();
            MarkNetworkUpdate();
        }
    }
    else if ( idx <= D6_ANGULAR_Z )
    {
        if ( enable != !EqualZero( angularServoEnable_.Data()[idx - 3]) )
        {
            (&angularServoEnable_.x_)[idx - 3] = enable?1:0;
            ApplyLimits();
            MarkNetworkUpdate();
        }
    }
}

void Constraint6DoF::SetServo()
{
    btGeneric6DofSpring2Constraint *gen6dofConstraint = static_cast<btGeneric6DofSpring2Constraint*>(constraint_.Get());
    gen6dofConstraint->setServo(D6_LINEAR_X, !EqualZero( linearServoEnable_.x_ ));
    gen6dofConstraint->setServo(D6_LINEAR_Y, !EqualZero( linearServoEnable_.y_ ));
    gen6dofConstraint->setServo(D6_LINEAR_Z, !EqualZero( linearServoEnable_.z_ ));
    gen6dofConstraint->setServo(D6_ANGULAR_X, !EqualZero( angularServoEnable_.x_ ));
    gen6dofConstraint->setServo(D6_ANGULAR_Y, !EqualZero( angularServoEnable_.y_ ));
    gen6dofConstraint->setServo(D6_ANGULAR_Z, !EqualZero( angularServoEnable_.z_ ));
}

void Constraint6DoF::SetEnableLinearMotor(const IntVector3& enable)
{
    if (enable != linearMotorEnable_)
    {
        linearMotorEnable_ = enable;

        ApplyLimits();
        MarkNetworkUpdate();
    }
}

const IntVector3& Constraint6DoF::GetEnableLinearMotor() const
{
    return linearMotorEnable_;
}

void Constraint6DoF::SetEnableAngularMotor(const IntVector3& enable)
{
    if (enable != angularMotorEnable_)
    {
        angularMotorEnable_ = enable;

        ApplyLimits();
        MarkNetworkUpdate();
    }
}

const IntVector3& Constraint6DoF::GetEnableAngularMotor() const
{
    return angularMotorEnable_;
}

void Constraint6DoF::SetEnableMotor(unsigned idx, bool on)
{
    if ( idx <= D6_LINEAR_Z )
    {
        if ( on != !EqualZero( linearMotorEnable_.Data()[idx] ) )
        {
            (&linearMotorEnable_.x_)[idx] = on?1:0;
            ApplyLimits();
            MarkNetworkUpdate();
        }
    }
    else if ( idx <= D6_ANGULAR_Z )
    {
        if ( on != !EqualZero( angularMotorEnable_.Data()[idx - 3] ) )
        {
            (&angularMotorEnable_.x_)[idx - 3] = on?1:0;
            ApplyLimits();
            MarkNetworkUpdate();
        }
    }
}

void Constraint6DoF::SetEnableMotor()
{
    btGeneric6DofSpring2Constraint *gen6dofConstraint = static_cast<btGeneric6DofSpring2Constraint*>(constraint_.Get());
    gen6dofConstraint->enableMotor(D6_LINEAR_X, !EqualZero( linearMotorEnable_.x_ ));
    gen6dofConstraint->enableMotor(D6_LINEAR_Y, !EqualZero( linearMotorEnable_.y_ ));
    gen6dofConstraint->enableMotor(D6_LINEAR_Z, !EqualZero( linearMotorEnable_.z_ ));
    gen6dofConstraint->enableMotor(D6_ANGULAR_X, !EqualZero( angularMotorEnable_.x_ ));
    gen6dofConstraint->enableMotor(D6_ANGULAR_Y, !EqualZero( angularMotorEnable_.y_ ));
    gen6dofConstraint->enableMotor(D6_ANGULAR_Z, !EqualZero( angularMotorEnable_.z_ ));
}

void Constraint6DoF::SetLinearMaxMotorForce(const Vector3& force)
{
    if ( force != linearMotorForce_ )
    {
        linearMotorForce_ = force;

        ApplyLimits();
        MarkNetworkUpdate();
    }
}

const Vector3& Constraint6DoF::GetLinearMaxMotorForce() const
{
    return linearMotorForce_;
}

void Constraint6DoF::SetAngularMaxMotorForce(const Vector3& force)
{
    if ( force != angularMotorForce_ )
    {
        angularMotorForce_ = force;

        ApplyLimits();
        MarkNetworkUpdate();
    }
}

const Vector3& Constraint6DoF::GetAngularMaxMotorForce() const
{
    return angularMotorForce_;
}

void Constraint6DoF::SetMaxMotorForce(unsigned idx, float force)
{
    if ( idx <= D6_LINEAR_Z )
    {
        (&linearMotorForce_.x_)[idx] = force;
        ApplyLimits();
        MarkNetworkUpdate();
    }
    else if ( idx <= D6_ANGULAR_Z )
    {
        (&angularMotorForce_.x_)[idx - 3] = force;
        ApplyLimits();
        MarkNetworkUpdate();
    }
}

void Constraint6DoF::SetMaxMotorForce()
{
    btGeneric6DofSpring2Constraint *gen6dofConstraint = static_cast<btGeneric6DofSpring2Constraint*>(constraint_.Get());
    gen6dofConstraint->setMaxMotorForce(D6_LINEAR_X, linearMotorForce_.x_);
    gen6dofConstraint->setMaxMotorForce(D6_LINEAR_Y, linearMotorForce_.y_);
    gen6dofConstraint->setMaxMotorForce(D6_LINEAR_Z, linearMotorForce_.z_);
    gen6dofConstraint->setMaxMotorForce(D6_ANGULAR_X, angularMotorForce_.x_);
    gen6dofConstraint->setMaxMotorForce(D6_ANGULAR_Y, angularMotorForce_.y_);
    gen6dofConstraint->setMaxMotorForce(D6_ANGULAR_Z, angularMotorForce_.z_);
}

void Constraint6DoF::SetLinearBounce(const Vector3& bounce)
{
    if (bounce != linearBounceForce_)
    {
        linearBounceForce_ = bounce;

        ApplyLimits();
        MarkNetworkUpdate();
    }
}

const Vector3& Constraint6DoF::GetLinearBounce() const
{
    return linearBounceForce_;
}

void Constraint6DoF::SetAngularBounce(const Vector3& bounce)
{
    if (bounce != angularBounceForce_)
    {
        angularBounceForce_ = bounce;

        ApplyLimits();
        MarkNetworkUpdate();
    }
}

const Vector3& Constraint6DoF::GetAngularBounce() const
{
    return angularBounceForce_;
}

void Constraint6DoF::SetBounce(unsigned idx, float bounce)
{
    if ( idx <= D6_LINEAR_Z )
    {
        (&linearBounceForce_.x_)[idx] = bounce;
        ApplyLimits();
        MarkNetworkUpdate();
    }
    else if ( idx <= D6_ANGULAR_Z )
    {
        (&angularBounceForce_.x_)[idx - 3] = bounce;
        ApplyLimits();
        MarkNetworkUpdate();
    }
}

void Constraint6DoF::SetBounce()
{
    btGeneric6DofSpring2Constraint *gen6dofConstraint = static_cast<btGeneric6DofSpring2Constraint*>(constraint_.Get());
    gen6dofConstraint->setBounce(D6_LINEAR_X,  linearBounceForce_.x_);
    gen6dofConstraint->setBounce(D6_LINEAR_Y,  linearBounceForce_.y_);
    gen6dofConstraint->setBounce(D6_LINEAR_Z,  linearBounceForce_.z_);
    gen6dofConstraint->setBounce(D6_ANGULAR_X, angularBounceForce_.x_);
    gen6dofConstraint->setBounce(D6_ANGULAR_Y, angularBounceForce_.y_);
    gen6dofConstraint->setBounce(D6_ANGULAR_Z, angularBounceForce_.z_);
}

void Constraint6DoF::SetEnableLinearSpring(const IntVector3& enable)
{
    if (enable != linearSpringEnable_)
    {
        linearSpringEnable_ = enable;

        ApplyLimits();
        MarkNetworkUpdate();
    }
}

const IntVector3& Constraint6DoF::GetEnableLinearSpring() const
{
    return linearSpringEnable_;
}

void Constraint6DoF::SetEnableAngularSpring(const IntVector3& enable)
{
    if (enable != angularSpringEnable_)
    {
        angularSpringEnable_ = enable;

        ApplyLimits();
        MarkNetworkUpdate();
    }
}

const IntVector3& Constraint6DoF::GetEnableAngularSpring() const
{
    return angularSpringEnable_;
}

void Constraint6DoF::SetEnableSpring(int idx, bool on)
{
    if ( idx <= D6_LINEAR_Z )
    {
        if (on != !EqualZero( linearSpringEnable_.Data()[idx]) )
        {
            (&linearSpringEnable_.x_)[idx] = on?1:0;
            ApplyLimits();
            MarkNetworkUpdate();
        }
    }
    else if ( idx <= D6_ANGULAR_Z )
    {
        if (on != !EqualZero( angularSpringEnable_.Data()[idx]) )
        {
            (&angularSpringEnable_.x_)[idx - 3] = on?1:0;
            ApplyLimits();
            MarkNetworkUpdate();
        }
    }
}

void Constraint6DoF::SetEnableSpring()
{
    btGeneric6DofSpring2Constraint *gen6dofConstraint = static_cast<btGeneric6DofSpring2Constraint*>(constraint_.Get());
    gen6dofConstraint->enableSpring(D6_LINEAR_X, !EqualZero( linearSpringEnable_.x_ ));  
    gen6dofConstraint->enableSpring(D6_LINEAR_Y, !EqualZero( linearSpringEnable_.y_ ));  
    gen6dofConstraint->enableSpring(D6_LINEAR_Z, !EqualZero( linearSpringEnable_.z_ ));  
    gen6dofConstraint->enableSpring(D6_ANGULAR_X, !EqualZero( angularSpringEnable_.x_ ));
    gen6dofConstraint->enableSpring(D6_ANGULAR_Y, !EqualZero( angularSpringEnable_.y_ ));
    gen6dofConstraint->enableSpring(D6_ANGULAR_Z, !EqualZero( angularSpringEnable_.z_ ));
}

void Constraint6DoF::SetLinearStiffness(const Vector3& stiffness)
{
    if ( stiffness != linearStiffness_ )
    {
        linearStiffness_ = stiffness;
        ApplyLimits();
        MarkNetworkUpdate();
    }
}

const Vector3& Constraint6DoF::GetLinearStiffness() const
{
    return linearStiffness_;
}

void Constraint6DoF::SetAngularStiffness(const Vector3& stiffness)
{
    if ( stiffness != linearStiffness_ )
    {
        angularStiffness_ = stiffness;

        ApplyLimits();
        MarkNetworkUpdate();
    }
}

const Vector3& Constraint6DoF::GetAngularStiffness() const
{
    return angularStiffness_;
}

void Constraint6DoF::SetStiffness(unsigned idx, float stiffness, bool limitIfNeeded)
{
    if ( idx <= D6_LINEAR_Z )
    {
        (&linearStiffness_.x_)[idx] = stiffness;
        (&linearStiffnessLimit_.x_)[idx] = limitIfNeeded?1:0;

        ApplyLimits();
        MarkNetworkUpdate();
    }
    else if ( idx <= D6_ANGULAR_Z )
    {
        (&angularStiffness_.x_)[idx] = stiffness;
        (&angularStiffnessLimit_.x_)[idx] = limitIfNeeded?1:0;

        ApplyLimits();
        MarkNetworkUpdate();
    }
}

void Constraint6DoF::SetLimitLinearStiffness(const IntVector3& limit)
{
    if ( limit != linearStiffnessLimit_ )
    {
        linearStiffnessLimit_ = limit;

        ApplyLimits();
        MarkNetworkUpdate();
    }
}

const IntVector3& Constraint6DoF::GetLimitLinearStiffness() const
{
    return linearStiffnessLimit_;
}

void Constraint6DoF::SetLimitAngularStiffness(const IntVector3& limit)
{
    if ( limit != angularStiffnessLimit_ )
    {
        angularStiffnessLimit_ = limit;

        ApplyLimits();
        MarkNetworkUpdate();
    }
}

const IntVector3& Constraint6DoF::GetLimitAngularStiffness() const
{
    return angularStiffnessLimit_;
}

void Constraint6DoF::SetStiffness()
{
    btGeneric6DofSpring2Constraint *gen6dofConstraint = static_cast<btGeneric6DofSpring2Constraint*>(constraint_.Get());
    gen6dofConstraint->setStiffness(D6_LINEAR_X,  linearStiffness_.x_, !EqualZero( linearStiffnessLimit_.x_ ));
    gen6dofConstraint->setStiffness(D6_LINEAR_Y,  linearStiffness_.y_, !EqualZero( linearStiffnessLimit_.y_ ));
    gen6dofConstraint->setStiffness(D6_LINEAR_Z,  linearStiffness_.z_, !EqualZero( linearStiffnessLimit_.z_ ));
    gen6dofConstraint->setStiffness(D6_ANGULAR_X, angularStiffness_.x_, !EqualZero( angularStiffnessLimit_.x_ ));
    gen6dofConstraint->setStiffness(D6_ANGULAR_Y, angularStiffness_.y_, !EqualZero( angularStiffnessLimit_.y_ ));
    gen6dofConstraint->setStiffness(D6_ANGULAR_Z, angularStiffness_.z_, !EqualZero( angularStiffnessLimit_.z_ ));
}

void Constraint6DoF::SetLinearDamping(const Vector3& damping)
{
    if ( damping != linearDamping_ )
    {
        linearDamping_ = damping;

        ApplyLimits();
        MarkNetworkUpdate();
    }
}

const Vector3& Constraint6DoF::GetLinearDamping() const
{
    return linearDamping_;
}

void Constraint6DoF::SetAngularDamping(const Vector3& damping)
{
    if ( damping != angularDamping_ )
    {
        angularDamping_ = damping;

        ApplyLimits();
        MarkNetworkUpdate();
    }
}

const Vector3& Constraint6DoF::GetAngularDamping() const
{
    return angularDamping_;
}

void Constraint6DoF::SetLimitLinearDamping(const IntVector3& limit)
{
    if ( limit != linearDampingLimit_ )
    {
        linearDampingLimit_ = limit;

        ApplyLimits();
        MarkNetworkUpdate();
    }
}

const IntVector3& Constraint6DoF::GetLimitLinearDamping() const
{
    return linearDampingLimit_;
}

void Constraint6DoF::SetLimitAngularDamping(const IntVector3& limit)
{
    if ( limit != angularDampingLimit_ )
    {
        angularDampingLimit_ = limit;

        ApplyLimits();
        MarkNetworkUpdate();
    }
}

const IntVector3& Constraint6DoF::GetLimitAngularDamping() const
{
    return angularDampingLimit_;
}

void Constraint6DoF::SetDamping(unsigned idx, float damping, bool limitIfNeeded)
{
    if ( idx <= D6_LINEAR_Z )
    {
        (&linearDamping_.x_)[idx] = damping;
        (&linearDampingLimit_.x_)[idx] = limitIfNeeded?1:0;

        ApplyLimits();
        MarkNetworkUpdate();
    }
    else if ( idx <= D6_ANGULAR_Z )
    {
        (&angularDamping_.x_)[idx] = damping;
        (&angularDampingLimit_.x_)[idx] = limitIfNeeded?1:0;

        ApplyLimits();
        MarkNetworkUpdate();
    }
}

void Constraint6DoF::SetDamping()
{
    btGeneric6DofSpring2Constraint *gen6dofConstraint = static_cast<btGeneric6DofSpring2Constraint*>(constraint_.Get());
    gen6dofConstraint->setDamping(D6_LINEAR_X,  linearDamping_.x_, !EqualZero( linearDampingLimit_.x_ ));
    gen6dofConstraint->setDamping(D6_LINEAR_Y,  linearDamping_.y_, !EqualZero( linearDampingLimit_.y_ ));
    gen6dofConstraint->setDamping(D6_LINEAR_Z,  linearDamping_.z_, !EqualZero( linearDampingLimit_.z_ ));
    gen6dofConstraint->setDamping(D6_ANGULAR_X, angularDamping_.x_, !EqualZero( angularDampingLimit_.x_ ));
    gen6dofConstraint->setDamping(D6_ANGULAR_Y, angularDamping_.y_, !EqualZero( angularDampingLimit_.y_ ));
    gen6dofConstraint->setDamping(D6_ANGULAR_Z, angularDamping_.z_, !EqualZero( angularDampingLimit_.z_ ));
}

void Constraint6DoF::SetEquilibriumPoint()
{
    btGeneric6DofSpring2Constraint *gen6dofConstraint = static_cast<btGeneric6DofSpring2Constraint*>(constraint_.Get());
    gen6dofConstraint->setEquilibriumPoint();
}

void Constraint6DoF::SetEquilibriumPoint(int idx)
{
    btGeneric6DofSpring2Constraint *gen6dofConstraint = static_cast<btGeneric6DofSpring2Constraint*>(constraint_.Get());
    gen6dofConstraint->setEquilibriumPoint(idx);
}

void Constraint6DoF::SetEquilibriumPoint(int idx, float val)
{
    btGeneric6DofSpring2Constraint *gen6dofConstraint = static_cast<btGeneric6DofSpring2Constraint*>(constraint_.Get());
    gen6dofConstraint->setEquilibriumPoint(idx, val);
}

void Constraint6DoF::SetLinearERPs(const Vector3 &erps)
{
    if (erps != linearErps_)
    {
        linearErps_ = erps;
        ApplyLimits();
        MarkNetworkUpdate();
    }
}

void Constraint6DoF::SetAngularERPs(const Vector3 &erps)
{
    if (erps != angularErps_)
    {
        angularErps_ = erps;
        ApplyLimits();
        MarkNetworkUpdate();
    }
}

void Constraint6DoF::SetERPs()
{
    btGeneric6DofSpring2Constraint *gen6dofConstraint = static_cast<btGeneric6DofSpring2Constraint*>(constraint_.Get());
    gen6dofConstraint->setParam(BT_CONSTRAINT_ERP, linearErps_.x_, 0);
    gen6dofConstraint->setParam(BT_CONSTRAINT_ERP, linearErps_.y_, 1);
    gen6dofConstraint->setParam(BT_CONSTRAINT_ERP, linearErps_.z_, 2);
    gen6dofConstraint->setParam(BT_CONSTRAINT_ERP, angularErps_.x_, 3);
    gen6dofConstraint->setParam(BT_CONSTRAINT_ERP, angularErps_.y_, 4);
    gen6dofConstraint->setParam(BT_CONSTRAINT_ERP, angularErps_.z_, 5);
}

void Constraint6DoF::SetLinearCFMs(const Vector3 &cmfs)
{
    if (cmfs != linearCfms_)
    {
        linearCfms_ = cmfs;
        ApplyLimits();
        MarkNetworkUpdate();
    }
}

void Constraint6DoF::SetAngularCFMs(const Vector3 &cmfs)
{
    if (cmfs != angularCfms_)
    {
        angularCfms_ = cmfs;
        ApplyLimits();
        MarkNetworkUpdate();
    }
}

void Constraint6DoF::SetCFMs()
{
    btGeneric6DofSpring2Constraint *gen6dofConstraint = static_cast<btGeneric6DofSpring2Constraint*>(constraint_.Get());
    gen6dofConstraint->setParam(BT_CONSTRAINT_CFM, linearCfms_.x_, 0);
    gen6dofConstraint->setParam(BT_CONSTRAINT_CFM, linearCfms_.y_, 1);
    gen6dofConstraint->setParam(BT_CONSTRAINT_CFM, linearCfms_.z_, 2);
    gen6dofConstraint->setParam(BT_CONSTRAINT_CFM, angularCfms_.x_, 3);
    gen6dofConstraint->setParam(BT_CONSTRAINT_CFM, angularCfms_.y_, 4);
    gen6dofConstraint->setParam(BT_CONSTRAINT_CFM, angularCfms_.z_, 5);
}

void Constraint6DoF::SetLinearStopERPs(const Vector3 &erps)
{
    if (erps != linearStopErps_)
    {
        linearStopErps_ = erps;
        ApplyLimits();
        MarkNetworkUpdate();
    }
}

void Constraint6DoF::SetAngularStopERPs(const Vector3 &erps)
{
    if (erps != angularStopErps_)
    {
        angularStopErps_ = erps;
        ApplyLimits();
        MarkNetworkUpdate();
    }
}

void Constraint6DoF::SetStopERPs()
{
    btGeneric6DofSpring2Constraint *gen6dofConstraint = static_cast<btGeneric6DofSpring2Constraint*>(constraint_.Get());
    gen6dofConstraint->setParam(BT_CONSTRAINT_STOP_ERP, linearStopErps_.x_, 0);
    gen6dofConstraint->setParam(BT_CONSTRAINT_STOP_ERP, linearStopErps_.y_, 1);
    gen6dofConstraint->setParam(BT_CONSTRAINT_STOP_ERP, linearStopErps_.z_, 2);
    gen6dofConstraint->setParam(BT_CONSTRAINT_STOP_ERP, angularStopErps_.x_, 3);
    gen6dofConstraint->setParam(BT_CONSTRAINT_STOP_ERP, angularStopErps_.y_, 4);
    gen6dofConstraint->setParam(BT_CONSTRAINT_STOP_ERP, angularStopErps_.z_, 5);
}

void Constraint6DoF::SetLinearStopCFMs(const Vector3 &cfms)
{
    if (cfms != linearStopCfms_)
    {
        linearStopCfms_ = cfms;
        ApplyLimits();
        MarkNetworkUpdate();
    }
}

void Constraint6DoF::SetAngularStopCFMs(const Vector3 &cfms)
{
    if (cfms != angularStopCfms_)
    {
        angularStopCfms_ = cfms;
        ApplyLimits();
        MarkNetworkUpdate();
    }
}

void Constraint6DoF::SetStopCFMs()
{
    btGeneric6DofSpring2Constraint *gen6dofConstraint = static_cast<btGeneric6DofSpring2Constraint*>(constraint_.Get());
    gen6dofConstraint->setParam(BT_CONSTRAINT_STOP_CFM, linearStopCfms_.x_, 0);
    gen6dofConstraint->setParam(BT_CONSTRAINT_STOP_CFM, linearStopCfms_.y_, 1);
    gen6dofConstraint->setParam(BT_CONSTRAINT_STOP_CFM, linearStopCfms_.z_, 2);
    gen6dofConstraint->setParam(BT_CONSTRAINT_STOP_CFM, angularStopCfms_.x_, 3);
    gen6dofConstraint->setParam(BT_CONSTRAINT_STOP_CFM, angularStopCfms_.y_, 4);
    gen6dofConstraint->setParam(BT_CONSTRAINT_STOP_CFM, angularStopCfms_.z_, 5);
}


}
