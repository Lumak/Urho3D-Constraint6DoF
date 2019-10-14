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

#pragma once

#include "../Math/Vector3.h"
#include "../Physics/Constraint.h"

namespace Urho3D
{

/// DoF 6 limit indices
enum D6LimitIndexType
{
    D6_LINEAR_X = 0,
    D6_LINEAR_Y,
    D6_LINEAR_Z,
    D6_ANGULAR_X,
    D6_ANGULAR_Y,
    D6_ANGULAR_Z,
};

class PhysicsWorld;
class RigidBody;

/// Physics constraint component. Connects two rigid bodies together, or one rigid body to a static point.
class URHO3D_API Constraint6DoF : public Constraint
{
    URHO3D_OBJECT(Constraint6DoF, Constraint);

    friend class RigidBody;

public:
    /// Construct.
    Constraint6DoF(Context* context);
    /// Destruct.
    ~Constraint6DoF();
    /// Register object factory.
    static void RegisterObject(Context* context);

    /// Handle attribute write access.
    virtual void OnSetAttribute(const AttributeInfo& attr, const Variant& src);
    /// Apply constraint frames.
    virtual void ApplyFrames();

    void SetLinearLowerLimit(const Vector3& linearLower);
    const Vector3& GetLinearLowerLimit() const;
    void SetLinearUpperLimit(const Vector3& linearUpper);
    const Vector3& GetLinearUpperLimit() const;
    void SetAngularLowerLimit(const Vector3& angularLower);
    const Vector3& GetAngularLowerLimit() const;
    void SetAngularUpperLimit(const Vector3& angularUpper);
    const Vector3& GetAngularUpperLimit() const;
    bool IsLimited(unsigned idx) const;

    /// set the type of the motor (servo or not) (the motor has to be turned on for servo also)
    void SetEnableLinearServo(const IntVector3& enable);
    const IntVector3& GetEnableLinearServo() const;
    void SetEnableAngularServo(const IntVector3& enable);
    const IntVector3& GetEnableAngularServo() const;
    void SetEnableServo(unsigned idx, bool enable);

    /// set the type of the motor (servo or not) (the motor has to be turned on for servo also)
    void SetEnableLinearMotor(const IntVector3& enable);
    const IntVector3& GetEnableLinearMotor() const;
    void SetEnableAngularMotor(const IntVector3& enable);
    const IntVector3& GetEnableAngularMotor() const;
    void SetEnableMotor(unsigned idx, bool on);

    void SetLinearMaxMotorForce(const Vector3& force);
    const Vector3& GetLinearMaxMotorForce() const;
    void SetAngularMaxMotorForce(const Vector3& force);
    const Vector3& GetAngularMaxMotorForce() const;
    void SetMaxMotorForce(unsigned idx, float force);

    void SetLinearBounce(const Vector3& bounce);
    const Vector3& GetLinearBounce() const;
    void SetAngularBounce(const Vector3& bounce);
    const Vector3& GetAngularBounce() const;
    void SetBounce(unsigned idx, float bounce);

    void SetEnableLinearSpring(const IntVector3& enable);
    const IntVector3& GetEnableLinearSpring() const;
    void SetEnableAngularSpring(const IntVector3& enable);
    const IntVector3& GetEnableAngularSpring() const;
    void SetEnableSpring(int idx, bool on);

    /// if limitIfNeeded is true the system will automatically limit the stiffness in necessary 
    /// situations where otherwise the spring would move unrealistically too widely
    void SetLinearStiffness(const Vector3& stiffness);
    const Vector3& GetLinearStiffness() const;
    void SetAngularStiffness(const Vector3& stiffness);
    const Vector3& GetAngularStiffness() const;
    void SetStiffness(unsigned idx, float stiffness, bool limitIfNeeded = true);
    void SetLimitLinearStiffness(const IntVector3& limit);
    const IntVector3& GetLimitLinearStiffness() const;
    void SetLimitAngularStiffness(const IntVector3& limit);
    const IntVector3& GetLimitAngularStiffness() const;

    /// if limitIfNeeded is true the system will automatically limit the damping in necessary situations 
    /// where otherwise the spring would blow up
    void SetLinearDamping(const Vector3& damping);
    const Vector3& GetLinearDamping() const;
    void SetAngularDamping(const Vector3& damping);
    const Vector3& GetAngularDamping() const;
    void SetDamping(unsigned idx, float damping, bool limitIfNeeded = true);
    void SetLimitLinearDamping(const IntVector3& limit);
    const IntVector3& GetLimitLinearDamping() const;
    void SetLimitAngularDamping(const IntVector3& limit);
    const IntVector3& GetLimitAngularDamping() const;

    /// Return constraint error reduction parameter.
    void SetLinearERPs(const Vector3 &erps);
    const Vector3& GetLinearERPs() const { return linearErps_; }
    void SetAngularERPs(const Vector3 &erps);
    const Vector3& GetAngularERPs() const { return angularErps_; }
    /// Return constraint force mixing parameter.
    void SetLinearCFMs(const Vector3 &cfms);
    const Vector3& GetLinearCFMs() const { return linearCfms_; }
    void SetAngularCFMs(const Vector3 &cfms);
    const Vector3& GetAngularCFMs() const { return angularCfms_; }

    /// Return constraint error reduction parameter.
    void SetLinearStopERPs(const Vector3 &erps);
    const Vector3& GetLinearStopERPs() const { return linearStopErps_; }
    void SetAngularStopERPs(const Vector3 &erps);
    const Vector3& GetAngularStopERPs() const { return angularStopErps_; }

    /// Return constraint force mixing parameter.
    void SetLinearStopCFMs(const Vector3 &cfms);
    const Vector3& GetLinearStopCFMs() const { return linearStopCfms_; }
    void SetAngularStopCFMs(const Vector3 &cfms);
    const Vector3& GetAngularStopCFMs() const { return angularStopCfms_; }

    /// set the current constraint position/orientation as an equilibrium point for all DOF
	void SetEquilibriumPoint();
    /// set the current constraint position/orientation as an equilibrium point for given DOF
	void SetEquilibriumPoint(int idx);
	void SetEquilibriumPoint(int idx, float val);

protected:
    /// Create the constraint.
    virtual void CreateConstraint();
    /// Apply high and low constraint limits.
    virtual void ApplyLimits();

    void SetLimits();
    void SetServo();
    void SetEnableMotor();
    void SetMaxMotorForce();
    void SetBounce();
    void SetEnableSpring();
    void SetStiffness();
    void SetDamping();
    void SetERPs();
    void SetCFMs();
    void SetStopERPs();
    void SetStopCFMs();

    inline bool EqualZero(int val) const { return (val == 0); }

protected:
    Vector3     linearLowerLimit_;
    Vector3     linearUpperLimit_;
    Vector3     angularLowerLimit_;
    Vector3     angularUpperLimit_;

    IntVector3  linearServoEnable_;
    IntVector3  angularServoEnable_;

    IntVector3  linearMotorEnable_;
    IntVector3  angularMotorEnable_;
    Vector3     linearMotorForce_;
    Vector3     angularMotorForce_;

    Vector3     linearBounceForce_;
    Vector3     angularBounceForce_;

    IntVector3  linearSpringEnable_;
    IntVector3  angularSpringEnable_;

    Vector3     linearStiffness_;
    Vector3     angularStiffness_;
    IntVector3  linearStiffnessLimit_;
    IntVector3  angularStiffnessLimit_;

    Vector3     linearDamping_;
    Vector3     angularDamping_;
    IntVector3  linearDampingLimit_;
    IntVector3  angularDampingLimit_;

    Vector3     linearErps_;
    Vector3     angularErps_;
    Vector3     linearCfms_;
    Vector3     angularCfms_;

    Vector3     linearStopErps_;
    Vector3     angularStopErps_;
    Vector3     linearStopCfms_;
    Vector3     angularStopCfms_;
};

}
