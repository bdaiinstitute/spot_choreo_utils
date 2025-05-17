# Copyright (c) 2025 Boston Dynamics AI Institute LLC. All rights reserved.

import json
import logging
import os
from pathlib import Path
from typing import Any, Dict, List, Union

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from web_animator.spot_web_animator.animation_backend.web_animator_serializer import (
    adjust_animation_timing,
    create_semantic_animation_builder,
    get_keyframe_times,
    load_animation_from_file,
    name_animation_segment,
    new_animation,
    save_pose,
    save_semantic_animation,
    update_keyframe,
    update_keyframe_time,
)

from spot_choreo_utils.choreo_creation.choreo_builders.animation_proto_utils import joint_angle_keyframe_to_proto
from spot_choreo_utils.choreo_creation.semantic_animations.semantic_animation_builder import SemanticAnimationBuilder
from spot_choreo_utils.paths import get_active_choreo_path
from spot_choreo_utils.serialization.serialization_utils import save_animation

# Set up logging
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
handler = logging.StreamHandler()
formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
handler.setFormatter(formatter)
logger.addHandler(handler)

# Create FastAPI app
app = FastAPI(title="Spot Dance Animator API")

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# Pydantic models for API requests
class NewAnimationRequest(BaseModel):
    name: str


class SaveKeyframeRequest(BaseModel):
    animation_name: str
    joint_angles: Dict[str, float]
    time: float


class UpdateKeyframeRequest(BaseModel):
    animation_name: str
    index: int
    joint_angles: Dict[str, float]
    time: float


class NameSegmentRequest(BaseModel):
    animation_name: str
    segment_name: str
    start_index: int
    end_index: int


class AdjustTimingRequest(BaseModel):
    animation_name: str
    keyframe_indices: List[int]
    new_timestamps: List[float]


# Global state
active_animation = None
active_semantic_builder = None
connected_websockets: set[WebSocket] = set()
spot = None
plant_context = None
context = None
meshcat = None
meshcat_url = "http://localhost:7000"
update_robot_function = None


# WebSocket connection manager
class ConnectionManager:
    def __init__(self) -> None:
        self.active_connections: List[WebSocket] = []
        self.spot = None
        self.plant_context = None
        self.context = None
        self.meshcat = None
        self.meshcat_url = "http://localhost:7000"
        self.active_animation = None

    async def connect(self, websocket: WebSocket) -> None:
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket) -> None:
        self.active_connections.remove(websocket)

    async def broadcast(self, message: Dict[str, Any]) -> None:
        for connection in self.active_connections:
            await connection.send_json(message)


manager = ConnectionManager()

def set_robot_references(
    spot_instance: Any, plant_context_instance: Any, context_instance: Any, meshcat_instance: Any
) -> None:
    """Set references to robot components needed for direct updates"""
    global spot, plant_context, context, meshcat, meshcat_url
    spot = spot_instance
    plant_context = plant_context_instance
    context = context_instance
    meshcat = meshcat_instance
    if hasattr(meshcat, "web_url"):
        meshcat_url = meshcat.web_url()
    elif hasattr(meshcat, "url"):
        meshcat_url = meshcat.url()
    logger.info(f"Using Meshcat URL: {meshcat_url}")


def update_robot_config_from_joint_angles(joint_angles: Dict[str, float]) -> bool:
    """Updates the robot configuration based on joint angles"""
    global spot, plant_context, context, update_robot_function
    if not spot or not plant_context or not context:
        logger.error("Robot references not set, cannot update pose")
        return False

    try:
        if update_robot_function:
            return update_robot_function(joint_angles, spot, plant_context)

        from web_animator.spot_web_animator.animation_backend.web_animator_backend import (
            update_robot_config_from_joint_angles as update_func,
        )

        update_func(joint_angles, spot, plant_context)
        return True
    except Exception as e:
        logger.error(f"Error updating robot pose: {e}")
        return False


# Get available animations
@app.get("/api/animations")
async def get_animations() -> Dict[str, Union[List[str], str]]:
    """Get list of available animations in the active folder"""
    try:
        animations = []
        active_path = get_active_choreo_path()

        # Find all animation folders
        for item in os.listdir(active_path):
            item_path = active_path / item
            if item_path.is_dir():
                animations.append(item)

        return {"animations": animations}
    except Exception as e:
        logger.error(f"Error getting animations: {e}")
        return {"error": str(e)}


# Create new animation
@app.post("/api/animations")
async def create_animation(request: NewAnimationRequest) -> Dict[str, Any]:
    """Create a new animation with the given name"""
    global active_animation, active_semantic_builder

    try:
        # Create initial keyframe with default pose
        idle_joint_angles = {
            "shoulder_0": 0.0,
            "shoulder_1": -3.12,
            "elbow_0": 3.13,
            "elbow_1": 1.57,
            "wrist_0": -0.00001,
            "wrist_1": -1.57,
            "gripper": -0.054,
            "front_left_hip_x": 0.12,
            "front_left_hip_y": 0.72,
            "front_left_knee": -1.45,
            "front_right_hip_x": -0.12,
            "front_right_hip_y": 0.72,
            "front_right_knee": -1.45,
            "rear_left_hip_x": 0.12,
            "rear_left_hip_y": 0.72,
            "rear_left_knee": -1.45,
            "rear_right_hip_x": -0.12,
            "rear_right_hip_y": 0.72,
            "rear_right_knee": -1.45,
            "body_pos_x": 0.0,
            "body_pos_y": 0.0,
            "body_pos_z": 0.525,
            "body_quat_w": 1.0,
            "body_quat_x": 0.0,
            "body_quat_y": 0.0,
            "body_quat_z": 0.0,
            "start_time": 0,
        }

        idle_keyframe = joint_angle_keyframe_to_proto(idle_joint_angles)
        active_animation = new_animation(idle_keyframe, 0, 0)
        active_animation.name = request.name

        # Save the animation
        path_to_animation = Path(get_active_choreo_path(), active_animation.name)
        save_animation(active_animation, path_to_animation)

        # Create semantic builder
        active_semantic_builder = create_semantic_animation_builder(active_animation)

        # Extract keyframe data for response
        keyframes = []
        for i, keyframe in enumerate(active_animation.animation_keyframes):
            keyframes.append({"index": i, "time": keyframe.time})

        return {"name": active_animation.name, "keyframes": keyframes}
    except Exception as e:
        logger.error(f"Error creating animation: {e}")
        return {"error": str(e)}


# Load animation
@app.get("/api/animations/{name}")
async def load_animation(name: str) -> Dict[str, Any]:
    """Load an existing animation by name"""
    global active_animation, active_semantic_builder

    try:
        animation_path = Path(get_active_choreo_path(), name, f"{name}.pbtxt")

        loaded_animation = load_animation_from_file(animation_path)
        if loaded_animation is not None:
            active_animation = loaded_animation

            # Try to load the semantic animation
            try:
                semantic_animation_path = Path(get_active_choreo_path(), name, f"{name}.pbtxt")
                active_semantic_builder = SemanticAnimationBuilder()
                active_semantic_builder.load_semantic_animation(semantic_animation_path)
            except Exception as e:
                logger.warning(f"Could not load semantic animation: {e}")
                active_semantic_builder = create_semantic_animation_builder(active_animation)

            # Extract keyframe data for response
            keyframes = []
            for i, keyframe in enumerate(active_animation.animation_keyframes):
                keyframes.append({"index": i, "time": keyframe.time})

            # Extract segment data if available
            segments = []
            if active_semantic_builder:
                segment_ranges = active_semantic_builder.get_segment_ranges()
                for name, (start, end) in segment_ranges.items():
                    segments.append({"name": name, "start_index": start, "end_index": end})

            return {"name": active_animation.name, "keyframes": keyframes, "segments": segments}
        else:
            return {"error": f"Animation {name} not found"}
    except Exception as e:
        logger.error(f"Error loading animation: {e}")
        return {"error": str(e)}


# Get keyframe data
@app.get("/api/animations/{name}/keyframes/{index}")
async def get_keyframe(name: str, index: int) -> Dict[str, Any]:
    """Get details of a specific keyframe"""
    global active_animation

    try:
        if active_animation and active_animation.name == name:
            if index < 0 or index >= len(active_animation.animation_keyframes):
                return {"error": f"Keyframe index {index} out of range"}

            keyframe = active_animation.animation_keyframes[index]

            # Extract joint angles from keyframe
            joint_angles = {}

            # Extract arm joint angles if available
            if keyframe.HasField("arm") and keyframe.arm.HasField("joint_angles"):
                arm = keyframe.arm.joint_angles
                if arm.HasField("shoulder_0"):
                    joint_angles["shoulder_0"] = arm.shoulder_0.value
                if arm.HasField("shoulder_1"):
                    joint_angles["shoulder_1"] = arm.shoulder_1.value
                if arm.HasField("elbow_0"):
                    joint_angles["elbow_0"] = arm.elbow_0.value
                if arm.HasField("elbow_1"):
                    joint_angles["elbow_1"] = arm.elbow_1.value
                if arm.HasField("wrist_0"):
                    joint_angles["wrist_0"] = arm.wrist_0.value
                if arm.HasField("wrist_1"):
                    joint_angles["wrist_1"] = arm.wrist_1.value

            # Extract gripper angle if available
            if keyframe.HasField("gripper") and keyframe.gripper.HasField("gripper_angle"):
                joint_angles["gripper"] = keyframe.gripper.gripper_angle.value

            # Extract body pose if available
            if keyframe.HasField("body"):
                body = keyframe.body
                if body.HasField("body_pos"):
                    pos = body.body_pos
                    if pos.HasField("x"):
                        joint_angles["body_pos_x"] = pos.x.value
                    if pos.HasField("y"):
                        joint_angles["body_pos_y"] = pos.y.value
                    if pos.HasField("z"):
                        joint_angles["body_pos_z"] = pos.z.value

                if body.HasField("quaternion"):
                    quat = body.quaternion
                    joint_angles["body_quat_w"] = quat.w
                    joint_angles["body_quat_x"] = quat.x
                    joint_angles["body_quat_y"] = quat.y
                    joint_angles["body_quat_z"] = quat.z

            # Extract leg angles if available
            if keyframe.HasField("legs"):
                legs = keyframe.legs

                if legs.HasField("fl") and legs.fl.HasField("joint_angles"):
                    fl = legs.fl.joint_angles
                    joint_angles["front_left_hip_x"] = fl.hip_x
                    joint_angles["front_left_hip_y"] = fl.hip_y
                    joint_angles["front_left_knee"] = fl.knee

                if legs.HasField("fr") and legs.fr.HasField("joint_angles"):
                    fr = legs.fr.joint_angles
                    joint_angles["front_right_hip_x"] = fr.hip_x
                    joint_angles["front_right_hip_y"] = fr.hip_y
                    joint_angles["front_right_knee"] = fr.knee

                if legs.HasField("hl") and legs.hl.HasField("joint_angles"):
                    hl = legs.hl.joint_angles
                    joint_angles["rear_left_hip_x"] = hl.hip_x
                    joint_angles["rear_left_hip_y"] = hl.hip_y
                    joint_angles["rear_left_knee"] = hl.knee

                if legs.HasField("hr") and legs.hr.HasField("joint_angles"):
                    hr = legs.hr.joint_angles
                    joint_angles["rear_right_hip_x"] = hr.hip_x
                    joint_angles["rear_right_hip_y"] = hr.hip_y
                    joint_angles["rear_right_knee"] = hr.knee

            return {"index": index, "time": keyframe.time, "joint_angles": joint_angles}
        else:
            return {"error": f"Animation {name} not active"}
    except Exception as e:
        logger.error(f"Error getting keyframe: {e}")
        return {"error": str(e)}


# Save keyframe
@app.post("/api/animations/{name}/keyframes")
async def save_keyframe(name: str, request: SaveKeyframeRequest) -> Dict[str, Any]:
    """Save a new keyframe to the animation"""
    global active_animation, active_semantic_builder

    try:
        if active_animation and active_animation.name == name:
            joint_angles = request.joint_angles.copy()
            joint_angles["start_time"] = request.time

            keyframe = joint_angle_keyframe_to_proto(joint_angles)
            if keyframe:
                save_pose(keyframe, len(active_animation.animation_keyframes), request.time, active_animation)

                # Save animation to disk
                path_to_animation = Path(get_active_choreo_path(), active_animation.name)
                save_animation(active_animation, path_to_animation)

                # Update semantic builder
                if active_semantic_builder:
                    active_semantic_builder.start_from_animation(active_animation)
                    save_semantic_animation(active_semantic_builder)

                # Extract keyframe data for response
                keyframes = []
                for i, keyframe in enumerate(active_animation.animation_keyframes):
                    keyframes.append({"index": i, "time": keyframe.time})

                return {"name": active_animation.name, "keyframes": keyframes}
            else:
                return {"error": "Failed to create keyframe"}
        else:
            return {"error": f"Animation {name} not active"}
    except Exception as e:
        logger.error(f"Error saving keyframe: {e}")
        return {"error": str(e)}


# Update keyframe
@app.put("/api/animations/{name}/keyframes/{index}")
async def update_keyframe_api(name: str, index: int, request: UpdateKeyframeRequest) -> Dict[str, Any]:
    """Update an existing keyframe"""
    global active_animation, active_semantic_builder

    try:
        if active_animation and active_animation.name == name:
            if index < 0 or index >= len(active_animation.animation_keyframes):
                return {"error": f"Keyframe index {index} out of range"}

            joint_angles = request.joint_angles.copy()
            joint_angles["start_time"] = request.time

            keyframe = joint_angle_keyframe_to_proto(joint_angles)
            if keyframe:
                # Update keyframe position and timing
                update_keyframe(active_animation, index, keyframe)
                update_keyframe_time(active_animation, index, request.time)

                # Save animation to disk
                path_to_animation = Path(get_active_choreo_path(), active_animation.name)
                save_animation(active_animation, path_to_animation)

                # Update semantic builder
                if active_semantic_builder:
                    active_semantic_builder.start_from_animation(active_animation)
                    save_semantic_animation(active_semantic_builder)

                return {"index": index, "time": active_animation.animation_keyframes[index].time}
            else:
                return {"error": "Failed to update keyframe"}
        else:
            return {"error": f"Animation {name} not active"}
    except Exception as e:
        logger.error(f"Error updating keyframe: {e}")
        return {"error": str(e)}


# Name segment
@app.post("/api/animations/{name}/segments")
async def name_segment(name: str, request: NameSegmentRequest) -> Dict[str, Any]:
    """Name a segment in the animation"""
    global active_animation, active_semantic_builder

    try:
        if active_animation and active_animation.name == name:
            if not active_semantic_builder:
                active_semantic_builder = create_semantic_animation_builder(active_animation)

            # Name the segment
            success = name_animation_segment(
                active_semantic_builder, request.segment_name, request.start_index, request.end_index
            )

            if success:
                # Save semantic animation
                save_semantic_animation(active_semantic_builder)

                # Extract segment data for response
                segments = []
                segment_ranges = active_semantic_builder.get_segment_ranges()
                for name, (start, end) in segment_ranges.items():
                    segments.append({"name": name, "start_index": start, "end_index": end})

                return {"segments": segments}
            else:
                return {"error": "Failed to name segment"}
        else:
            return {"error": f"Animation {name} not active"}
    except Exception as e:
        logger.error(f"Error naming segment: {e}")
        return {"error": str(e)}


# Adjust timing
@app.put("/api/animations/{name}/timing")
async def adjust_timing(name: str, request: AdjustTimingRequest) -> Dict[str, Any]:
    """Adjust the timing of keyframes"""
    global active_animation, active_semantic_builder

    try:
        if active_animation and active_animation.name == name:
            # Validate the request
            if len(request.keyframe_indices) != len(request.new_timestamps):
                return {"error": "Keyframe indices and timestamps must be the same length"}

            if len(request.keyframe_indices) < 2:
                return {"error": "At least 2 keyframe indices are required"}

            # Adjust timing
            success = adjust_animation_timing(active_animation, request.keyframe_indices, request.new_timestamps)

            if success:
                # Save animation to disk
                path_to_animation = Path(get_active_choreo_path(), active_animation.name)
                save_animation(active_animation, path_to_animation)

                # Update semantic builder
                if active_semantic_builder:
                    active_semantic_builder.start_from_animation(active_animation)
                    save_semantic_animation(active_semantic_builder)

                # Extract keyframe times for response
                keyframe_times = get_keyframe_times(active_animation)

                return {"keyframe_times": keyframe_times}
            else:
                return {"error": "Failed to adjust timing"}
        else:
            return {"error": f"Animation {name} not active"}
    except Exception as e:
        logger.error(f"Error adjusting timing: {e}")
        return {"error": str(e)}


# Play animation
@app.post("/api/animations/{name}/play")
async def play_animation(name: str) -> Dict[str, Any]:
    """Play animation in Meshcat visualization"""
    global active_animation, manager

    try:
        if active_animation and active_animation.name == name:
            # Send play command to connected WebSocket clients
            await manager.broadcast({"type": "play_animation", "animation_name": name})

            return {"status": "playing"}
        else:
            return {"error": f"Animation {name} not active"}
    except Exception as e:
        logger.error(f"Error playing animation: {e}")
        return {"error": str(e)}


# Stop animation
@app.post("/api/animations/{name}/stop")
async def stop_animation(name: str) -> Dict[str, Any]:
    """Stop animation playback"""
    global active_animation, manager

    try:
        if active_animation and active_animation.name == name:
            # Send stop command to connected WebSocket clients
            await manager.broadcast({"type": "stop_animation", "animation_name": name})

            return {"status": "stopped"}
        else:
            return {"error": f"Animation {name} not active"}
    except Exception as e:
        logger.error(f"Error stopping animation: {e}")
        return {"error": str(e)}


def set_active_animation(animation: Any) -> None:
    """Set the currently active animation"""
    global active_animation
    active_animation = animation


@app.get("/api/current-animation")
async def get_current_animation() -> Dict[str, Any]:
    """Get the currently active animation"""
    global active_animation
    if active_animation is not None:
        keyframes = [{"index": i, "time": kf.time} for i, kf in enumerate(active_animation.animation_keyframes)]
        return {"name": active_animation.name, "keyframes": keyframes}
    return {"name": None, "keyframes": []}


# WebSocket endpoint for real-time communication
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket) -> None:
    print("WebSocket connection attempt received")
    await manager.connect(websocket)
    try:
        while True:
            print("Waiting for WebSocket message")
            data = await websocket.receive_text()
            print(f"Received message: {data}")

            if "get_meshcat_url" in data:
                print("Received meshcat URL request, sending response")
                await websocket.send_json({"type": "meshcat_url", "url": meshcat_url})
                continue

            try:
                message = json.loads(data)

                if message["type"] == "update_robot_pose":
                    # Get the joint angles
                    joint_angles = message["joint_angles"]

                    # Use the update function that was passed in
                    success = update_robot_config_from_joint_angles(joint_angles)

                    # Broadcast to all clients
                    await manager.broadcast(
                        {"type": "robot_pose_updated", "joint_angles": joint_angles, "success": success}
                    )
                elif message["type"] == "get_meshcat_url":
                    await websocket.send_json({"type": "meshcat_url", "url": meshcat_url})
            except json.JSONDecodeError:
                logger.error(f"Invalid JSON: {data}")
    except WebSocketDisconnect:
        manager.disconnect(websocket)


# Run the server
if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=8000)
