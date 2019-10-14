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

namespace Urho3D
{
class Node;
class Scene;
class SplinePath;
}
using namespace Urho3D;

//=============================================================================
//=============================================================================
class Constraint6DoFHelper : public LogicComponent
{
    URHO3D_OBJECT(Constraint6DoFHelper, LogicComponent);
    
public:
    Constraint6DoFHelper(Context* context);
    virtual ~Constraint6DoFHelper();

    static void RegisterObject(Context* context);
    /// Called on scene update, variable timestep.
    virtual void Update(float timeStep);
    /// Called on scene post-update, variable timestep.
    virtual void PostUpdate(float timeStep);
    /// Called on physics update, fixed timestep.
    virtual void FixedUpdate(float timeStep);
    /// Called on physics post-update, fixed timestep.
    virtual void FixedPostUpdate(float timeStep);

    void SetSplinePathNode(Node *node);
    void SetEnableRailMover(bool enable);
protected:
    void UpdateMover(float timeStep);

protected:
    WeakPtr<Node> splinePathNode_;
    WeakPtr<SplinePath> splinePath_;
    WeakPtr<Node> controlNode_;
    bool railMoverEnabled_;

};





