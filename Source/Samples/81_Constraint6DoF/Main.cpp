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
#include <Urho3D/Core/CoreEvents.h>
#include <Urho3D/Engine/Engine.h>
#include <Urho3D/Graphics/AnimatedModel.h>
#include <Urho3D/Graphics/Camera.h>
#include <Urho3D/Graphics/DebugRenderer.h>
#include <Urho3D/Graphics/Graphics.h>
#include <Urho3D/Graphics/Light.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Octree.h>
#include <Urho3D/Graphics/Renderer.h>
#include <Urho3D/Graphics/Zone.h>
#include <Urho3D/Input/Input.h>
#include <Urho3D/IO/File.h>
#include <Urho3D/IO/FileSystem.h>
#include <Urho3D/IO/Log.h>
#include <Urho3D/Physics/CollisionShape.h>
#include <Urho3D/Physics/PhysicsWorld.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/Physics/Constraint6DoF.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Scene/SplinePath.h>
#include <Urho3D/UI/Font.h>
#include <Urho3D/UI/Text.h>
#include <Urho3D/UI/UI.h>

#include "Main.h"
#include "Constraint6DoFHelper.h"

#include <Urho3D/DebugNew.h>

//=============================================================================
//=============================================================================
URHO3D_DEFINE_APPLICATION_MAIN(Main)

//=============================================================================
//=============================================================================
Main::Main(Context* context)
    : Sample(context)
    , camFreeMode_(true)
    , controlSpeed_(0.0f)
    , drawDebug_(false)
{
    context->RegisterFactory<Constraint6DoFHelper>();
}

void Main::Setup()
{
    engineParameters_["WindowTitle"]   = GetTypeName();
    engineParameters_["LogName"]       = GetSubsystem<FileSystem>()->GetProgramDir() + "6Dof.log";
    engineParameters_["FullScreen"]    = false;
    engineParameters_["Headless"]      = false;
    engineParameters_["WindowWidth"]   = 1280; 
    engineParameters_["WindowHeight"]  = 720;
    engineParameters_["ResourcePaths"] = "Data;CoreData;Data/6DoF;";
}

void Main::Start()
{
    // Execute base class startup
    Sample::Start();

    // Create the scene content
    CreateScene();

    // Setup the viewport for displaying the scene
    SetupViewport();

    // Hook up to the frame update and render post-update events
    SubscribeToEvents();

    // Set the mouse mode to use in the sample
    Sample::InitMouseMode(MM_ABSOLUTE);
}

void Main::CreateScene()
{
    scene_ = new Scene(context_);

    // Create the camera. Limit far clip distance to match the fog. Note: now we actually create the camera node outside
    // the scene, because we want it to be unaffected by scene load / save
    cameraNode_ = new Node(context_);
    Camera* camera = cameraNode_->CreateComponent<Camera>();
    camera->SetFarClip(300.0f);

    // Set an initial position for the camera scene node above the floor
    cameraNode_->SetPosition(Vector3(0.0f, 1.0f, -10.0f));

    LoadScene(0);
}

void Main::LoadScene(unsigned idx)
{
    ResourceCache* cache = GetSubsystem<ResourceCache>();

    // load scene
    XMLFile *xmlLevel = cache->GetResource<XMLFile>("6DoF/Scenes/Scene0.xml");
    scene_->LoadXML(xmlLevel->GetRoot());

    // Set instruction text
    // Create the UI content
    if (instructionText_ == NULL)
    {
        CreateInstructions();
    }
    instructionText_->SetText("");

    const String& varName = scene_->GetVarName(StringHash("SceneInfo"));
    if (!varName.Empty())
    {
        String strInfo = scene_->GetVar(StringHash("SceneInfo")).GetString();
        instructionText_->SetText(strInfo);
    }

    // Clear
    sceneControlRigidbody_ = NULL;
    sceneConstraint6DoF_ = NULL;
    controlSpeed_ = 0.0f;
    Node *controlNode = NULL;

    sceneSplinePath_ = scene_->GetComponent<SplinePath>(true);

    if (sceneSplinePath_)
    {
        controlSpeed_ = sceneSplinePath_->GetSpeed();
        Node *cnode = scene_->CreateChild();
        Constraint6DoFHelper *c6DoFHelper = cnode->CreateComponent<Constraint6DoFHelper>();

        c6DoFHelper->SetSplinePathNode(sceneSplinePath_->GetNode() );
        c6DoFHelper->SetEnableRailMover(true);
    }

    // Get rigidbody
    if ((controlNode = scene_->GetChild("controlConstraint6DoFNode", true)) != NULL)
    {
        sceneConstraint6DoF_ = controlNode->GetComponent<Constraint6DoF>();
    }

    if ((controlNode = scene_->GetChild("controlRigidBody", true)) != NULL)
    {
        sceneControlRigidbody_ = controlNode->GetComponent<RigidBody>();
    }

    // Setup cam node
    sceneCamLookAtNode_ = NULL;
    if ((sceneCamNode_ = scene_->GetChild("camNode", true)) != NULL)
    {
        sceneCamLookAtNode_ = sceneCamNode_->GetParent();
        camFreeMode_ = false;
    }
}

void Main::CreateInstructions()
{
    ResourceCache* cache = GetSubsystem<ResourceCache>();
    UI* ui = GetSubsystem<UI>();

    // Construct new Text object, set string to display and font to use
    instructionText_ = ui->GetRoot()->CreateChild<Text>();
    //instructionText_->SetText(
    //    "Use WASD keys and mouse/touch to move\n"
    //    "LMB to spawn physics objects\n"
    //    "F5 to save scene, F7 to load\n"
    //    "Space to toggle physics debug geometry"
    //);
    instructionText_->SetFont(cache->GetResource<Font>("Fonts/Anonymous Pro.ttf"), 12);
    // The text has multiple rows. Center them in relation to each other
    instructionText_->SetTextAlignment(HA_CENTER);

    // Position the text relative to the screen center
    instructionText_->SetHorizontalAlignment(HA_CENTER);
    instructionText_->SetPosition(0, 20);
}

void Main::SetupViewport()
{
    Renderer* renderer = GetSubsystem<Renderer>();

    // Set up a viewport to the Renderer subsystem so that the 3D scene can be seen
    SharedPtr<Viewport> viewport(new Viewport(context_, scene_, cameraNode_->GetComponent<Camera>()));
    renderer->SetViewport(0, viewport);
}

void Main::MoveCamera(float timeStep)
{
    // Do not move if the UI has a focused element (the console)
    if (GetSubsystem<UI>()->GetFocusElement())
        return;

    Input* input = GetSubsystem<Input>();

    // Movement speed as world units per second
    const float MOVE_SPEED = 20.0f;
    // Mouse sensitivity as degrees per pixel
    const float MOUSE_SENSITIVITY = 0.1f;

    // Use this frame's mouse motion to adjust camera node yaw and pitch. Clamp the pitch between -90 and 90 degrees
    IntVector2 mouseMove = input->GetMouseMove();
    yaw_ += MOUSE_SENSITIVITY * mouseMove.x_;
    pitch_ += MOUSE_SENSITIVITY * mouseMove.y_;
    pitch_ = Clamp(pitch_, -90.0f, 90.0f);

    // Construct new orientation for the camera scene node from yaw and pitch. Roll is fixed to zero
    cameraNode_->SetRotation(Quaternion(pitch_, yaw_, 0.0f));

    // Read WASD keys and move the camera scene node to the corresponding direction if they are pressed
    if (input->GetKeyDown(KEY_W))
        cameraNode_->Translate(Vector3::FORWARD * MOVE_SPEED * timeStep);
    if (input->GetKeyDown(KEY_S))
        cameraNode_->Translate(Vector3::BACK * MOVE_SPEED * timeStep);
    if (input->GetKeyDown(KEY_A))
        cameraNode_->Translate(Vector3::LEFT * MOVE_SPEED * timeStep);
    if (input->GetKeyDown(KEY_D))
        cameraNode_->Translate(Vector3::RIGHT * MOVE_SPEED * timeStep);

    // "Shoot" a physics object with left mousebutton
    if (input->GetMouseButtonPress(MOUSEB_LEFT))
        SpawnObject();

    // Check for loading / saving the scene
    if (input->GetKeyPress(KEY_F5))
    {
    }
    if (input->GetKeyPress(KEY_F7))
    {
    }
}

void Main::SpawnObject()
{
    ResourceCache* cache = GetSubsystem<ResourceCache>();

    Node* boxNode = scene_->CreateChild("Sphere");
    boxNode->SetPosition(cameraNode_->GetPosition());
    boxNode->SetRotation(cameraNode_->GetRotation());
    boxNode->SetScale(0.3f);
    StaticModel* boxObject = boxNode->CreateComponent<StaticModel>();
    boxObject->SetModel(cache->GetResource<Model>("Models/Sphere.mdl"));
    boxObject->SetMaterial(cache->GetResource<Material>("Materials/StoneSmall.xml"));
    boxObject->SetCastShadows(true);

    RigidBody* body = boxNode->CreateComponent<RigidBody>();
    body->SetCollisionLayer(1);
    body->SetMass(1.0f);
    body->SetRollingFriction(0.1f);
    body->SetAngularDamping(0.3f);
    body->SetRestitution(0.5f);

    CollisionShape* shape = boxNode->CreateComponent<CollisionShape>();
    shape->SetSphere(1.0f);

    const float OBJECT_VELOCITY = 14.0f;

    // Set initial velocity for the RigidBody based on camera forward vector. Add also a slight up component
    // to overcome gravity better
    body->SetLinearVelocity(cameraNode_->GetRotation() * Vector3(0.0f, 0.25f, 1.0f) * OBJECT_VELOCITY);
}

void Main::SubscribeToEvents()
{
    // Subscribe HandleUpdate() function for processing update events
    SubscribeToEvent(E_UPDATE, URHO3D_HANDLER(Main, HandleUpdate));

    // Subscribe HandlePostRenderUpdate() function for processing the post-render update event, during which we request
    // debug geometry
    SubscribeToEvent(E_POSTRENDERUPDATE, URHO3D_HANDLER(Main, HandlePostRenderUpdate));
}

void Main::HandleUpdate(StringHash eventType, VariantMap& eventData)
{
    using namespace Update;

    // Take the frame time step, which is stored as a float
    float timeStep = eventData[P_TIMESTEP].GetFloat();
    Input* input = GetSubsystem<Input>();

    if (sceneCamNode_ && input->GetKeyPress(KEY_C))
    {
        camFreeMode_ = !camFreeMode_;
    }

    // Move the camera, scale movement with time step
    if (camFreeMode_)
    {
        MoveCamera(timeStep);
    }
    else
    {
        MoveControlBody(timeStep);
    }

    // Toggle physics debug geometry with space
    if (input->GetKeyPress(KEY_SPACE))
        drawDebug_ = !drawDebug_;
}

void Main::MoveControlBody(float timeStep)
{
    Input* input = GetSubsystem<Input>();

    if (sceneControlRigidbody_)
    {
        const float LINEAR_FORCE = 40.0f;
        const float ANGULAR_FORCE = 0.4f;
        Quaternion rot = sceneControlRigidbody_->GetRotation();

        if (sceneConstraint6DoF_)
        {
            sceneConstraint6DoF_->SetAngularLowerLimit(Vector3::ZERO);
            sceneConstraint6DoF_->SetAngularUpperLimit(Vector3::ZERO);
        }

        if (input->GetKeyDown(KEY_W))
        {
            sceneControlRigidbody_->ApplyForce(rot * Vector3::UP * LINEAR_FORCE);
        }
        if (input->GetKeyDown(KEY_S))
        {
            sceneControlRigidbody_->ApplyForce(rot * Vector3::DOWN * LINEAR_FORCE);
        }
        if (input->GetKeyDown(KEY_A))
        {
            Vector3 force = rot * Vector3::LEFT * LINEAR_FORCE;
            force.y_ = 0.0f;

            sceneControlRigidbody_->ApplyForce(force);
            sceneControlRigidbody_->ApplyTorque(rot* Vector3::FORWARD * ANGULAR_FORCE);
            sceneConstraint6DoF_->SetAngularLowerLimit(Vector3(0,0,-45));
        }
        if (input->GetKeyDown(KEY_D))
        {
            Vector3 force = rot * Vector3::RIGHT * LINEAR_FORCE;
            force.y_ = 0.0f;
            sceneControlRigidbody_->ApplyForce(force);
            sceneControlRigidbody_->ApplyTorque(rot* Vector3::BACK * ANGULAR_FORCE);
            sceneConstraint6DoF_->SetAngularUpperLimit(Vector3(0,0,45));
        }
    }

    cameraNode_->SetPosition(sceneCamNode_->GetWorldPosition());
    cameraNode_->SetRotation(sceneCamNode_->GetWorldRotation());
    cameraNode_->LookAt(sceneCamLookAtNode_->GetWorldPosition());
}

void Main::HandlePostRenderUpdate(StringHash eventType, VariantMap& eventData)
{
    // If draw debug mode is enabled, draw physics debug geometry. Use depth test to make the result easier to interpret
    if (drawDebug_)
    {
        scene_->GetComponent<PhysicsWorld>()->DrawDebugGeometry(true);

        DebugRenderer *dbgRenderer = scene_->GetComponent<DebugRenderer>();
        if (dbgRenderer)
        {
            sceneSplinePath_->DrawDebugGeometry(dbgRenderer, true);
        }
    }
}


