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
#include "Sample.h"

namespace Urho3D
{
class Node;
class Scene;
class RigidBody;
class Constraint6DoF;
class SplinePath;
class BorderImage;
}

using namespace Urho3D;

class HoverBike;
//=============================================================================
//=============================================================================
class Main : public Sample
{
    URHO3D_OBJECT(Main, Sample);
    
public:
    /// Construct.
    Main(Context* context);
    
    /// Setup after engine initialization and before running the main loop.
    virtual void Setup();
    virtual void Start();

protected:
    /// Construct the scene content.
    void CreateScene();
    void LoadScene(unsigned idx);
    /// Construct an instruction text to the UI.
    void CreateInstructions();
    /// Set up a viewport for displaying the scene.
    void SetupViewport();
    /// Subscribe to application-wide logic update and post-render update events.
    void SubscribeToEvents();
    /// Read input and moves the camera.
    void MoveCamera(float timeStep);
    /// Handle the logic update event.
    void HandleUpdate(StringHash eventType, VariantMap& eventData);
    /// Handle the post-render update event.
    void HandlePostRenderUpdate(StringHash eventType, VariantMap& eventData);
    void MoveShipOnRail(float timeStep);
    void MoveHoverBike(float timeStep);

    void LoadSceneNext();
    void UpdateLoadScene();

protected:
    WeakPtr<Text> instructionText_;
    WeakPtr<Node> loadedScene_;
    WeakPtr<Node> sceneCamNode_;
    WeakPtr<Node> sceneCamLookAtNode_;
    WeakPtr<SplinePath> sceneSplinePath_;
    WeakPtr<RigidBody> sceneControlRigidbody_;
    WeakPtr<Constraint6DoF> sceneConstraint6DoF_;
    WeakPtr<HoverBike> sceneHoverBike_;
    float controlSpeed_;

    bool camFreeMode_;

    /// Flag for drawing debug geometry.
    bool drawDebug_;

    enum LoadState
    {
        LOAD_INIT,
        LOAD_START,
        LOAD_FILE_BEGIN,
    };
    enum MoverType
    {
        MOVER_SHIP,
        MOVER_HOVERBIKE,
    };
    Vector<String> loadFilenames_;
    WeakPtr<BorderImage> loadScreenImage_;
    unsigned loadFileIdx_;
    unsigned loadState_;
    unsigned loadStateTicks_;
};
