import warp as wp
import numpy as np
import torch
from typing import List, Dict, Optional, Union


class SceneManager:
    def __init__(self, batch_size: int = 1, device: str = "cuda:0"):
        """Initialize the scene manager with batch support

        Args:
            batch_size: Number of parallel environments/simulations
            device: Torch device string (e.g. "cuda:0", "cuda:1")
        """
        wp.init()
        self.batch_size = batch_size
        self.device = device
        self.meshes = []  # List of warp meshes in the scene
        self.mesh_ids = None  # Combined mesh IDs array
        self.objects = []  # List of objects in scene: [{"position": vec3, "radius": float}]
        self.target_pos = None

    def create_box_mesh(self, size=1.0, flip_normals=False, size_x=None, size_y=None, size_z=None):
        """Create a box mesh with given size
        
        Args:
            size: Size of the box (used for all dimensions if specific sizes not provided)
            flip_normals: Whether to flip face normals (reverse winding order)
            size_x: Specific size for x-dimension
            size_y: Specific size for y-dimension
            size_z: Specific size for z-dimension
        """
        # Use specific sizes if provided, otherwise use general size
        sx = size_x if size_x is not None else size
        sy = size_y if size_y is not None else size
        sz = size_z if size_z is not None else size
        
        vertices = np.array([
            [-sx, -sy, -sz],
            [sx, -sy, -sz],
            [sx, sy, -sz],
            [-sx, sy, -sz],
            [-sx, -sy, sz],
            [sx, -sy, sz],
            [sx, sy, sz],
            [-sx, sy, sz]
        ], dtype=np.float32)

        faces = np.array([
            [0,1,2], [0,2,3],  # bottom
            [4,5,6], [4,6,7],  # top
            [0,1,5], [0,5,4],  # front
            [2,3,7], [2,7,6],  # back
            [1,2,6], [1,6,5],  # right
            [0,3,7], [0,7,4]   # left
        ], dtype=np.int32)

        if flip_normals:
            # Reverse winding order to flip normals
            faces = np.flip(faces, axis=1)

        mesh = wp.Mesh(
            points=wp.array(vertices, dtype=wp.vec3),
            indices=wp.array(faces.flatten(), dtype=wp.int32),
            velocities=wp.zeros(len(vertices), dtype=wp.vec3)
        )
        return mesh

    def create_sphere_mesh(self, radius=1.0, flip_normals=False):
        """Create a sphere mesh with given radius
        
        Args:
            radius: Radius of the sphere
            flip_normals: Whether to flip face normals (reverse winding order)
        """
        subdivisions = 16
        vertices = []
        for i in range(subdivisions + 1):
            theta = i * np.pi / subdivisions
            for j in range(subdivisions):
                phi = j * 2 * np.pi / subdivisions
                x = radius * np.sin(theta) * np.cos(phi)
                y = radius * np.sin(theta) * np.sin(phi)
                z = radius * np.cos(theta)
                vertices.append([x, y, z])
        
        faces = []
        for i in range(subdivisions):
            for j in range(subdivisions):
                p1 = i * subdivisions + j
                p2 = i * subdivisions + (j + 1) % subdivisions
                p3 = (i + 1) * subdivisions + j
                p4 = (i + 1) * subdivisions + (j + 1) % subdivisions
                
                if i != 0:
                    faces.append([p1, p2, p3])
                if i != subdivisions - 1:
                    faces.append([p3, p2, p4])
        
        vertices = np.array(vertices, dtype=np.float32)
        faces = np.array(faces, dtype=np.int32)

        if flip_normals:
            # Reverse winding order to flip normals
            faces = np.flip(faces, axis=1)
        
        mesh = wp.Mesh(
            points=wp.array(vertices, dtype=wp.vec3),
            indices=wp.array(faces.flatten(), dtype=wp.int32),
            velocities=wp.zeros(len(vertices), dtype=wp.vec3)
        )
        return mesh
    
    def generate_target_pos(self):
        """Generate a target position for the scene
        
        Args:
            num_targets: Number of target positions to generate
        """
        x = torch.rand((self.batch_size, 1), device=self.device) * 0.1 + 0.2
        y = torch.rand((self.batch_size, 1), device=self.device) * 0.2 + 0.4
        z = torch.rand((self.batch_size, 1), device=self.device) * 0.5 + 1.3

        target_pos = torch.cat([x, y, z], dim=1)
        self.target_pos = target_pos

    def wall_pos(self):
        x = np.random.uniform(0.0, 3.0)
        y = np.random.uniform(-2, 2)
        z = 0.0 #np.random.uniform(-0.2, 0.2) + 0.4

        pos = np.array([x, y, z])
        return pos

    def random_pos(self):
        x = np.random.uniform(-self.room_size*0.5, self.room_size*0.5)
        y = np.random.uniform(-self.room_size*0.5, self.room_size*0.5)
        z = np.random.uniform(1, self.room_size)

        pos = np.array([x, y, z])
        return pos

    
    def setup_room(self, room_size = 10.0, num_objects=2, rand_obs=True, obs_size = 1.0, obs_type = 1.0, obs_dim = 2.0):
        #if rand_obs = True, means we randomize the location of obstacles in the room. 
        #If obs_size is -1, that means we randomize the size of obstacles

        self.room_size = room_size
        """Setup a room with random obstacles
        
        Args:
            room_size: Size of the room (half-extent)
            num_objects: Number of random objects to add
        """
        # Clear existing objects # List of objects in scene: [{"position": vec3, "radius": float}]
        self.objects.clear()
        
        # Initialize combined mesh data
        all_points = []
        all_indices = []
        all_velocities = []
        vertex_count = 0

        # Create room and add to combined mesh. Use a box to create the room
        room = self.create_box_mesh(size=100000, flip_normals=True)
        room_points = room.points.numpy()
        room_indices = room.indices.numpy()
        room_velocities = room.velocities.numpy()
        
        all_points.extend(room_points)
        all_indices.extend(room_indices)
        all_velocities.extend(room_velocities)
        vertex_count += len(room_points)

        # Add random objects
        for i in range(num_objects):
            if obs_type == 3.0:
                if np.random.rand() > 0.5:
                    size = np.random.uniform(0.1, obs_size)
                    if obs_dim == 2.0:
                        obj = self.create_box_mesh(size=size, size_z = room_size, flip_normals=True)
                    elif obs_dim == 3.0:
                        obj = self.create_box_mesh(size=size, flip_normals=True)
                    radius = size  # Approximate box with sphere
                    obj_type = "box"
                else:
                    radius = np.random.uniform(0.1, obs_size)
                    obj = self.create_sphere_mesh(radius=radius, flip_normals=True)
                    obj_type = "sphere"
            elif obs_type == 2.0:
                radius = np.random.uniform(0.1, obs_size)
                obj = self.create_sphere_mesh(radius=radius, flip_normals=True)
                obj_type = "sphere"
            elif obs_type == 1.0:
                size = np.random.uniform(0.5, obs_size) / 2. #TODO: To change back to random if doing random size
                # size = 1.0
                # print(f"Obstacle size radius is {size}")
                if obs_dim == 2.0:
                    obj = self.create_box_mesh(size=size, size_z = 100000, flip_normals=True)
                elif obs_dim == 3.0:
                    obj = self.create_box_mesh(size=size, flip_normals=True)
                radius = np.sqrt(2*(size*size))  # Approximate box with sphere
                # print(f"Obstacle actual size radius is {size}")
                obj_type = "box"

            # Specify the centre of the obstacle in the obstacle-attached frame
            if rand_obs == True:
                pos = self.random_pos()
            else:
                #TODO: specify in the middle of the room first. To make sure I pass in a list of objects in future.
                pos = self.random_pos() * 0.0 #+ np.array([3,0,0])

            #Update the obstacles points in global frame
            obj_points = obj.points.numpy() + pos
            obj_indices = obj.indices.numpy() + vertex_count  #Add the indices according to combined obstacle vertices
            obj_velocities = obj.velocities.numpy()   #Amend obstacle velocities accordingly.
            
            # Store object info for collision detection. TODO: NOTE THAT THE RADIUS IS IMPORTANT FOR COMPUTING distance to obstacle.
            self.objects.append({
                "position": pos,
                "radius": radius,
                "type": obj_type,
            })
            
            all_points.extend(obj_points)  #Note that these are global frame info.
            all_indices.extend(obj_indices)
            all_velocities.extend(obj_velocities)
            vertex_count += len(obj_points)
            

        # Create single combined mesh
        combined_mesh = wp.Mesh(
            points=wp.array(all_points, dtype=wp.vec3),
            indices=wp.array(all_indices, dtype=wp.int32),
            velocities=wp.array(all_velocities, dtype=wp.vec3)
        )
        self.add_mesh(combined_mesh)
        
    def setup_poisson_room(self, room_size = 10.0, num_objects=2, rand_obs=True, obs_size = 1.0, obs_dim = 2.0, min_obs_gap=0.0):
        #if rand_obs = True, means we randomize the location of obstacles in the room. 
        #If obs_size is -1, that means we randomize the size of obstacles
        ####NOTE: The obstacles are generated from [0,0] to [+ve room_size , +ve room_size]

        self.room_size = room_size
        """Setup a room with random obstacles
        
        Args:
            room_size: Size of the room (half-extent)
            num_objects: Number of random objects to add
        """
        # Clear existing objects # List of objects in scene: [{"position": vec3, "radius": float}]
        self.objects.clear()
        
        # Initialize combined mesh data
        all_points = []
        all_indices = []
        all_velocities = []
        vertex_count = 0

        # Create room and add to combined mesh. Use a box to create the room
        room = self.create_box_mesh(size=100000, flip_normals=True)
        room_points = room.points.numpy()
        room_indices = room.indices.numpy()
        room_velocities = room.velocities.numpy()
        
        all_points.extend(room_points)
        all_indices.extend(room_indices)
        all_velocities.extend(room_velocities)
        vertex_count += len(room_points)

        ##Generate location and radii of obstacles first
        # Use fixed seed when rand_obs=False for consistent obstacle placement across training/testing
        seed = None if rand_obs else 42
        centers, radii = self.generate_obstacles(room_w=room_size, room_h=room_size, min_r=0.5, max_r=obs_size, n_obs=num_objects, max_tries=50, min_gap=min_obs_gap, seed=seed)

        # Add random objects
        for i in range(num_objects):
            size = radii[i] #TODO: To change back to random if doing random size
            if obs_dim == 2.0:
                obj = self.create_box_mesh(size=size, size_z = 100000, flip_normals=True)
            elif obs_dim == 3.0:
                obj = self.create_box_mesh(size=size, flip_normals=True)
            radius = np.sqrt(2*(size*size))  # Approximate box with sphere
            # print(f"Obstacle actual size radius is {size}")
            obj_type = "box"

            # Specify the centre of the obstacle in the obstacle-attached frame
            z = np.random.uniform(-5, 5)
            pos = np.zeros((1,3))
            pos[:,0] = centers[i][0]
            pos[:,1] = centers[i][1]
            pos[:,2] = z

            #Update the obstacles points in global frame
            obj_points = obj.points.numpy() + pos
            obj_indices = obj.indices.numpy() + vertex_count  #Add the indices according to combined obstacle vertices
            obj_velocities = obj.velocities.numpy()   #Amend obstacle velocities accordingly.
            
            # Store object info for collision detection. TODO: NOTE THAT THE RADIUS IS IMPORTANT FOR COMPUTING distance to obstacle.
            self.objects.append({
                "position": pos,
                "radius": radius,
                "type": obj_type,
            })
            
            all_points.extend(obj_points)  #Note that these are global frame info.
            all_indices.extend(obj_indices)
            all_velocities.extend(obj_velocities)
            vertex_count += len(obj_points)
            

        # Create single combined mesh
        combined_mesh = wp.Mesh(
            points=wp.array(all_points, dtype=wp.vec3),
            indices=wp.array(all_indices, dtype=wp.int32),
            velocities=wp.array(all_velocities, dtype=wp.vec3)
        )
        self.add_mesh(combined_mesh)

        return centers, radii

    def setup_room_from_obstacles(self, room_size=10.0, obstacles=None, obs_dim=2.0):
        """Build a room whose obstacles are specified explicitly (e.g. loaded
        from a saved training config) instead of being sampled via Poisson-disc.

        Args:
            room_size: Room size (kept for consistency with the other setups).
            obstacles: Either a dict (e.g. {"obs1": {...}, ...}) or a list of
                       dicts, each with:
                         'position': [x, y, z]
                         'radius'  : sphere-approx radius (== box_size * sqrt(2))
                         'type'    : optional label, defaults to 'box'
            obs_dim: 2.0 -> tall vertical columns, 3.0 -> finite boxes. Matches
                     the convention used in setup_poisson_room.

        Returns:
            (centers, radii): centers is (N, 2) [x, y]; radii is (N,) the
            sphere-approx radius per obstacle (same value stored in self.objects
            and consumed by create_obs_vec / sample_safe_points_in_region).
        """
        self.room_size = room_size
        self.objects.clear()

        all_points = []
        all_indices = []
        all_velocities = []
        vertex_count = 0

        # Enclosing room (same convention as setup_poisson_room)
        room = self.create_box_mesh(size=100000, flip_normals=True)
        room_points = room.points.numpy()
        all_points.extend(room_points)
        all_indices.extend(room.indices.numpy())
        all_velocities.extend(room.velocities.numpy())
        vertex_count += len(room_points)

        # Normalize the obstacle container into an ordered list of dicts
        if isinstance(obstacles, dict):
            obs_items = [obstacles[k] for k in sorted(obstacles.keys())]
        else:
            obs_items = list(obstacles) if obstacles is not None else []

        centers = []
        radii = []
        for obs in obs_items:
            px, py, pz = [float(v) for v in obs["position"]]
            radius = float(obs["radius"])            # sphere-approx radius
            size = radius / np.sqrt(2.0)             # recover box half-size
            obj_type = obs.get("type", "box")

            if obs_dim == 2.0:
                obj = self.create_box_mesh(size=size, size_z=100000, flip_normals=True)
            else:
                obj = self.create_box_mesh(size=size, flip_normals=True)

            pos = np.zeros((1, 3))
            pos[:, 0] = px
            pos[:, 1] = py
            pos[:, 2] = pz

            obj_points = obj.points.numpy() + pos
            obj_indices = obj.indices.numpy() + vertex_count
            obj_velocities = obj.velocities.numpy()

            self.objects.append({
                "position": pos,
                "radius": radius,
                "type": obj_type,
            })

            all_points.extend(obj_points)
            all_indices.extend(obj_indices)
            all_velocities.extend(obj_velocities)
            vertex_count += len(obj_points)

            centers.append((px, py))
            radii.append(radius)

        combined_mesh = wp.Mesh(
            points=wp.array(all_points, dtype=wp.vec3),
            indices=wp.array(all_indices, dtype=wp.int32),
            velocities=wp.array(all_velocities, dtype=wp.vec3),
        )
        self.add_mesh(combined_mesh)

        return np.array(centers), np.array(radii)


    def add_mesh(self, mesh: wp.Mesh) -> None:
        """Add a mesh to the scene
        
        Args:
            mesh: Warp mesh object to add to the scene
        """
        self.meshes.append(mesh)
        self._update_mesh_ids()
        
    def remove_mesh(self, mesh: wp.Mesh) -> None:
        """Remove a mesh from the scene
        
        Args:
            mesh: Warp mesh object to remove
        """
        if mesh in self.meshes:
            self.meshes.remove(mesh)
            self._update_mesh_ids()
            
    def _update_mesh_ids(self) -> None:
        """Update the combined mesh IDs array"""
        # print(f"Updating mesh_ids with {len(self.meshes)} meshes")
        if self.meshes:
            self.mesh_ids = wp.array([m.id for m in self.meshes], dtype=wp.uint64)
        else:
            self.mesh_ids = None

    def sample_safe_points(self, n_points=100, min_dist=0.5):
        """
        Sample points within room bounds but outside obstacle area by at least `min_dist`.

        Args:
            bounds: ((xmin, ymin), (xmax, ymax))
            obstacle_center: (x, y)
            obstacle_size: (width, height)
            n_points: number of samples
            min_dist: minimum distance from obstacle boundary
        """

        bounds=[[-self.room_size, -self.room_size, 0], [self.room_size, self.room_size, 3.0]]
        xmin, ymin, _ = bounds[0]
        xmax, ymax, _ = bounds[1]

        obs_centre_list = []
        obs_size_list = []

        for ii in self.objects:
            obs_centre_list.append(ii["position"])
            obs_size_list.append(ii["radius"])
        # ox, oy = obstacle_center
        # ow, oh = obstacle_size
        obs_centre_array = np.array(obs_centre_list)
        obs_size_array = np.array(obs_size_list)

        safe_points = []

        while len(safe_points) < n_points:
            # Sample uniformly in the room
            x = np.random.uniform(xmin, xmax)
            y = np.random.uniform(ymin, ymax)
            z = np.random.uniform(0,bounds[-1][-1])
            if len(obs_centre_list) == 0:
                safe_points.append([x, y, z])
            else:
                sample_point = np.array([x,y,z])

                #### TODO: Assumed box and infinite height for now. To make it general
                a = np.linalg.norm(obs_centre_array[:,:2] - sample_point[:2], axis=1)
                chk_dist = a > obs_size_array
                total_pass = np.sum(chk_dist).item()

                # Compute distance from obstacle edges

                # dx = max(abs(x - ox) - ow/2, 0.0)
                # dy = max(abs(y - oy) - oh/2, 0.0)
                # dist = np.sqrt(dx**2 + dy**2)

                # Check if outside obstacle + margin
                if total_pass == obs_centre_array.shape[0]:
                    safe_points.append([x, y, z])

        return np.array(safe_points, dtype=np.float32)

    def generate_obstacles(self, room_w, room_h, min_r, max_r, n_obs, max_tries=50, min_gap=0.0, seed=None):
        centers = []
        radii = []
        tries = 0

        # Use RandomState for reproducibility when seed is provided
        if seed is not None:
            rng = np.random.RandomState(seed)
        else:
            rng = np.random

        while len(centers) < n_obs and tries < max_tries * n_obs:
            tries += 1
            r = rng.uniform(min_r, max_r)
            cx = rng.uniform(r, room_w - r)
            cy = rng.uniform(r, room_h - r)

            ok = True
            for (px, py), pr in zip(centers, radii):
                if np.hypot(cx - px, cy - py) < np.sqrt(2) * (r + pr + min_gap):
                    ok = False
                    break
            if ok:
                centers.append((cx, cy))
                radii.append(r)

        return np.array(centers), np.array(radii)


    # -------------------------------------------------------------
    # Sample safe points inside a chosen sub-region
    # -------------------------------------------------------------
    def sample_safe_points_in_region(self,room_region, obstacles_center, obstacles_radius,
                                    d_safe, n_points, max_tries=1000):

        (x_min, x_max, y_min, y_max) = room_region
        pts = []
        tries = 0

        while len(pts) < n_points and tries < max_tries * n_points:
            tries += 1

            # sample inside region
            x = np.random.uniform(x_min, x_max)
            y = np.random.uniform(y_min, y_max)

            ok = True
            for (cx, cy), r in zip(obstacles_center, obstacles_radius):
                if np.hypot(x - cx, y - cy) < (r + d_safe):
                    ok = False
                    break

            if ok:
                z = np.random.uniform(1.0, 2.0)
                pts.append((x, y, z))

        return np.array(pts)


    def get_nearest_object_distance(self, position: torch.Tensor, temperature: float = 0.1) -> tuple:
        """Calculate differentiable distance vector to nearest object's collision sphere
        
        Args:
            position: 3D position(s) to check from [..., 3] or [batch_size, ..., 3]
            temperature: Softmax temperature for softmin operation (lower = sharper)
            
        Returns:
            tuple: (distance_vector, distance_magnitude) to nearest object
        """
        if not self.objects:
            return (
                torch.zeros_like(position, dtype=torch.float32),
                torch.zeros(
                    position.shape[:-1],
                    device=position.device,
                    dtype=torch.float32,
                    requires_grad=True
                )
            )
            
        # Convert objects to tensors with matching dtype and device
        obj_positions = torch.stack([
            torch.as_tensor(obj["position"],
                          device=position.device,
                          dtype=torch.float32)
            for obj in self.objects
        ])  # [num_objects, 3]
        
        obj_radii = torch.as_tensor(
            [obj["radius"] for obj in self.objects],
            device=position.device,
            dtype=torch.float32
        )  # [num_objects]
        
        # Ensure position is float32 if it isn't already
        position = position.to(dtype=torch.float32)
        
        # Calculate vectors from positions to objects [..., num_objects, 3]
        vecs = obj_positions - position.unsqueeze(-2)
        
        # Calculate distances [..., num_objects]
        dists = torch.norm(vecs, dim=-1) - obj_radii
        
        # Softmin weights [..., num_objects]
        weights = torch.nn.functional.softmin(dists / temperature, dim=-1)
        
        # Weighted average of distances and vectors
        nearest_dists = torch.sum(weights * dists, dim=-1)
        nearest_vecs = torch.sum(weights.unsqueeze(-1) * vecs, dim=-2)
        # Normalize vectors and scale by distance where distance > 0
        mask = nearest_dists > 0
        if mask.any():
            # Create new tensor instead of inplace modification
            norm_vecs = nearest_vecs / (torch.norm(nearest_vecs, dim=-1, keepdim=True) + 1e-8)
            scaled_vecs = norm_vecs * nearest_dists.unsqueeze(-1)
            nearest_vecs = torch.where(mask.unsqueeze(-1), scaled_vecs, nearest_vecs)
            nearest_vecs[mask] = norm_vecs[mask] * nearest_dists[mask].unsqueeze(-1)
            
        nearest_dists = nearest_dists.unsqueeze(-1)
        return nearest_vecs, nearest_dists

    def get_nearest_object_hard_max_distance(self, position: torch.Tensor, temperature: float = 0.1) -> tuple:

        ############NOTE That I ONLY CRAFT THIS FOR 2D objects for now. This outputs unit vector
        """Calculate differentiable distance vector to nearest object's collision sphere
        
        Args:
            position: 3D position(s) to check from [..., 3] or [batch_size, ..., 3]
            temperature: Softmax temperature for softmin operation (lower = sharper)
            
        Returns:
            tuple: (distance_vector, distance_magnitude) to nearest object
        """
        if not self.objects:
            return (
                torch.zeros_like(position, dtype=torch.float32),
                torch.zeros(
                    position.shape[:-1],
                    device=position.device,
                    dtype=torch.float32,
                    requires_grad=True
                )
            )
            
        # Convert objects to tensors with matching dtype and device
        obj_positions = torch.stack([
            torch.as_tensor(obj["position"],
                          device=position.device,
                          dtype=torch.float32)
            for obj in self.objects
        ])  # [num_objects, 3]
        
        obj_radii = torch.as_tensor(
            [obj["radius"] for obj in self.objects],
            device=position.device,
            dtype=torch.float32
        )  # [num_objects]

        # obj_radii = obj_radii * torch.sqrt(torch.tensor(2.0, device=position.device, dtype=obj_radii.dtype))
        
        # Ensure position is float32 if it isn't already
        position = position.to(dtype=torch.float32)

        drones = position.squeeze(1) ##[batch_size x 3]

        obstacles = obj_positions.squeeze(1) ##[num_obstacles x 3]

        diff = obstacles[None, :, :] - drones[:, None, :] ## [batch_size x num_obstacles x 3]
        distances = torch.norm(diff[:,:,:2], dim=2) - obj_radii ## [ batch_size x num_objects]

        nearest_dists, min_indices = distances.min(dim=1)  ## [batch_size x 1]

        nearest_vecs = diff[torch.arange(drones.size(0)), min_indices, :] #[batch_size x 3]
        nearest_vecs[:,-1] = 0
        eps = 1e-8
        nearest_vecs = nearest_vecs / (nearest_dists.unsqueeze(1) + eps)

        return nearest_vecs, nearest_dists
        
    def save_scene(self, filepath: str) -> None:
        """Save current scene state to JSON file
        
        Args:
            filepath: Path to save scene file
        """
        import json
        import numpy as np
        
        # Convert numpy arrays to strings for JSON serialization
        def convert_arrays(obj):
            if isinstance(obj, np.ndarray):
                return obj.tolist()  # Convert to list which is JSON serializable
            elif isinstance(obj, dict):
                return {k: convert_arrays(v) for k, v in obj.items()}
            elif isinstance(obj, list):
                return [convert_arrays(item) for item in obj]
            return obj
            
        scene_data = {
            "objects": convert_arrays(self.objects),
        }
        
        with open(filepath, 'w') as f:
            json.dump(scene_data, f, indent=2)
            
    def load_scene(self, filepath: str) -> None:
        """Load scene state from JSON file
        
        Args:
            filepath: Path to scene file to load
        """
        import json
        import numpy as np
        
        with open(filepath, 'r') as f:
            scene_data = json.load(f)
            
        # Convert lists back to numpy arrays
        def restore_arrays(obj):
            if isinstance(obj, dict):
                if 'position' in obj:  # Convert position list to numpy array
                    obj['position'] = np.array(obj['position'])
                return {k: restore_arrays(v) for k, v in obj.items()}
            elif isinstance(obj, list):
                return [restore_arrays(item) for item in obj]
            return obj
            
        scene_data = restore_arrays(scene_data)
            
        # Clear current scene
        self.objects.clear()
        self.meshes.clear()
        
        # Load object list
        self.objects = scene_data["objects"]
        
        # Rebuild meshes for all objects
        for obj in self.objects:
            # Create simple sphere mesh for each object
            # (Assuming objects have 'radius' property)
            self.create_box_mesh(size=obj.get("radius", 1.0))
        
if __name__ == "__main__":
    import os
    from flyinglib.simulation.utils import *
    import yaml 
    home_dir = os.path.expanduser("~")
    config = os.path.join(home_dir,"intern_storage/velocity_obstacle/perceptive_diff_drone/examples/config/diff_sim_velocity_tracking_nwu_config.yaml")
    with open(config, 'r') as file:
        loaded_params = yaml.safe_load(file)

    config_params = parse_args(config_path=config)

    DEVICE = f"cuda:{config_params.GPU}"
    torch.cuda.set_device(config_params.GPU)
    torch.set_default_tensor_type('torch.cuda.FloatTensor')

    env_manager = SceneManager(batch_size=config_params.env_copy, device=DEVICE)
    centers, radii = env_manager.setup_poisson_room(room_size=config_params.room_size, num_objects=config_params.num_obstacles, rand_obs=config_params.to_randomize_obs_location,
                                    obs_size=config_params.obs_size, obs_dim=config_params.obs_dim, min_obs_gap=config_params.min_obs_gap)