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
#include <Urho3D/Graphics/AnimatedModel.h>
#include <Urho3D/Graphics/DebugRenderer.h>
#include <Urho3D/Physics/CollisionShape.h>
#include <Urho3D/Physics/PhysicsWorld.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/Physics/Constraint6DoF.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Scene/SplinePath.h>

#include "Constraint6DoFHelper.h"

#include <Urho3D/DebugNew.h>
//=============================================================================
//=============================================================================
Constraint6DoFHelper::Constraint6DoFHelper(Context* context)
    : LogicComponent(context)
    , railMoverEnabled_(false)
{
    SetUpdateEventMask(0);
}

Constraint6DoFHelper::~Constraint6DoFHelper()
{
}

void Constraint6DoFHelper::RegisterObject(Context* context)
{
    context->RegisterFactory<Constraint6DoFHelper>();
}

void Constraint6DoFHelper::SetSplinePathNode(Node *node)
{
    splinePathNode_ = node;
    splinePath_ = NULL;

    if (splinePathNode_)
    {
        splinePath_ = splinePathNode_->GetComponent<SplinePath>();

        controlNode_ = splinePath_->GetControlledNode();
    }
}

void Constraint6DoFHelper::SetEnableRailMover(bool enable)
{
    if (enable != railMoverEnabled_)
    {
        railMoverEnabled_ = enable;

        if (railMoverEnabled_)
        {
            SetUpdateEventMask(USE_UPDATE);
        }
        else
        {
            SetUpdateEventMask(0);
        }
    }
}

void Constraint6DoFHelper::Update(float timeStep)
{
    UpdateMover(timeStep);
}

void Constraint6DoFHelper::PostUpdate(float timeStep)
{
}

void Constraint6DoFHelper::FixedUpdate(float timeStep)
{
}

void Constraint6DoFHelper::FixedPostUpdate(float timeStep)
{
}

void Constraint6DoFHelper::UpdateMover(float timeStep)
{
    if (splinePath_)
    {
        splinePath_->Move(timeStep);

        // Looped path, reset to continue
        if (splinePath_->IsFinished())
        {
            splinePath_->Reset();
        }

        // Orient the control node in the path direction
        if (controlNode_)
        {
            Vector3 curPos = splinePath_->GetPosition();
            float traveled = splinePath_->GetTraveled() + 0.02f;
            if (traveled > 1.0f)
            {
                traveled -= 1.0f;
            }
            Vector3 aheadPos = splinePath_->GetPoint(traveled);
            Vector3 dir = aheadPos - curPos;
            dir.y_ = 0.0f;
            dir.Normalized();
            controlNode_->SetDirection(dir);
        }
    }
}











