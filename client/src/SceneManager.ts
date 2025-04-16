import * as THREE from 'three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import * as SkeletonUtils from 'three/examples/jsm/utils/SkeletonUtils.js';
import * as moduleBindings from './module_bindings/index';

// Animation-related constants
const ANIMATION_FADE_DURATION = 0.2; // seconds
const CAMERA_DISTANCE = 2; // Distance behind the character
const CAMERA_HEIGHT = 1; // Height above the character

// --- Type Definitions ---
// Client-side animation state: Only idle or walking
export type AnimationName = 'idle' | 'walk';
// Server-side/Network animation state: Includes direction
export type ServerAnimationState = 'idle' | 'walkingForwards' | 'walkingBackwards';

// Represents the mutable Three.js components associated with a player model
export interface PlayerRenderComponent {
    model: THREE.Group; // The root Group node for this player
    mixer: THREE.AnimationMixer;
    actions: {
        idle: THREE.AnimationAction;
        walk: THREE.AnimationAction; // Single action for both forward/backward walking
    };
    currentAnimation: AnimationName;
    lastPositionSq: THREE.Vector2; // Use Vector2 for 2D checks, store squared distance
}

// Interface for local player state data needed for rendering
export interface LocalPlayerRenderData {
    readonly identity: string;
    readonly position: THREE.Vector3;
    readonly rotationY: number;
    readonly pitch: number;
    readonly currentAnimationName: AnimationName;
    readonly isMovingBackwards: boolean;
    readonly hexColor?: string; // Optional hex color for player customization
}

// Interface for remote player state data needed for rendering
export interface RemotePlayerRenderData {
    readonly identity: string;
    readonly position: { readonly x: number, readonly y: number }; // From server, y is z in 3D space
    readonly rotationYaw: number;
    readonly animationState: string; // Server animation state
    readonly hexColor?: string; // Optional hex color for player customization
}

// Callback for when all assets are loaded
export type AssetsLoadedCallback = () => void;

export class SceneManager {
    private scene: THREE.Scene;
    private camera: THREE.PerspectiveCamera;
    private renderer: THREE.WebGLRenderer;
    private loader: GLTFLoader;
    private canvasContainer: HTMLDivElement;
    
    // Assets
    private characterModelTemplate: THREE.Group | null = null;
    private stadiumModel: THREE.Group | null = null;
    private idleClips: THREE.AnimationClip[] = [];
    private walkClips: THREE.AnimationClip[] = [];
    
    // Render components
    private playerRenderComponents: Map<string, PlayerRenderComponent> = new Map();
    private localPlayerRenderComponent: PlayerRenderComponent | null = null;
    
    // Debug helpers
    private gridHelper: THREE.GridHelper | null = null;
    private axesHelper: THREE.AxesHelper | null = null;
    private debugInfoElement: HTMLDivElement | null = null;
    private isDebugActive: boolean = false;
    
    // State tracking
    private isDisposed: boolean = false;
    private assetsLoaded: { stadium: boolean, template: boolean, idle: boolean, walk: boolean } = { 
        stadium: false, template: false, idle: false, walk: false 
    };
    private onAssetsLoadedCallback: AssetsLoadedCallback | null = null;

    constructor(canvasContainer: HTMLDivElement) {
        this.canvasContainer = canvasContainer;
        
        // Initialize Three.js Core
        this.scene = new THREE.Scene();
        this.camera = new THREE.PerspectiveCamera(
            75,
            canvasContainer.clientWidth / canvasContainer.clientHeight,
            0.1,
            1000
        );
        this.camera.position.set(0, 1, 10); // Default starting position
        this.renderer = new THREE.WebGLRenderer({ antialias: true });
        this.loader = new GLTFLoader();
        
        // Setup Scene
        this.scene.background = new THREE.Color(0x87ceeb); // Sky blue
        this.addLighting();
        
        // Setup Renderer
        this.renderer.setSize(canvasContainer.clientWidth, canvasContainer.clientHeight);
        this.renderer.setPixelRatio(window.devicePixelRatio);
        canvasContainer.appendChild(this.renderer.domElement);
        
        // Create and setup debug info element
        this.setupDebugOverlay();
        
        // Setup window resize handler
        window.addEventListener('resize', this.handleResize);
    }
    
    // Load all required assets
    public loadAssets(onAssetsLoaded: AssetsLoadedCallback): void {
        this.onAssetsLoadedCallback = onAssetsLoaded;
        
        // Load Stadium
        this.loader.load('/models/forest_scene/scene.glb', (gltf) => {
            if (this.isDisposed) return;
            this.stadiumModel = gltf.scene;
            this.stadiumModel.scale.set(1, 1, 1);
            this.stadiumModel.position.set(0, 0, 0);
            if (!this.scene) {
                console.warn("Scene is null during stadium load callback, likely disposed.");
                return;
            }
            this.scene.add(this.stadiumModel);
            this.assetsLoaded.stadium = true; 
            this.checkAllAssetsLoaded();
        }, undefined, (error) => console.error('Error loading stadium:', error));

        // Load Character Template
        this.loader.load('/models/character/XBot.glb', (gltf) => {
            if (this.isDisposed) return;
            this.characterModelTemplate = gltf.scene;
            this.assetsLoaded.template = true; 
            this.checkAllAssetsLoaded();
        }, undefined, (error) => console.error('Error loading character template:', error));

        // Load Idle Animation
        this.loader.load('/models/character/animations/Idle.glb', (gltf) => {
            if (this.isDisposed) return;
            this.idleClips = gltf.animations;
            this.assetsLoaded.idle = true; 
            this.checkAllAssetsLoaded();
        }, undefined, (error) => console.error('Error loading idle animation:', error));

        // Load Walk Animation
        this.loader.load('/models/character/animations/Walking.glb', (gltf) => {
            if (this.isDisposed) return;
            this.walkClips = gltf.animations;
            this.assetsLoaded.walk = true; 
            this.checkAllAssetsLoaded();
        }, undefined, (error) => console.error('Error loading walking animation:', error));
    }
    
    private checkAllAssetsLoaded(): void {
        if (this.isDisposed) return;
        if (Object.values(this.assetsLoaded).every(Boolean)) {
            if (this.onAssetsLoadedCallback) {
                this.onAssetsLoadedCallback();
            }
        }
    }
    
    public areRenderAssetsReady(): boolean {
        return !!this.characterModelTemplate && this.idleClips.length > 0 && this.walkClips.length > 0;
    }
    
    // Setup lighting and debug helpers
    private addLighting(): void {
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.7);
        this.scene.add(ambientLight);
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.5);
        directionalLight.position.set(5, 10, 7.5);
        this.scene.add(directionalLight);
            
        // Add a grid to help with orientation
        this.gridHelper = new THREE.GridHelper(20, 20, 0xff0000, 0xffffff);
        this.gridHelper.visible = false;
        this.scene.add(this.gridHelper);
        
        // Add 3D coordinate axes
        this.axesHelper = new THREE.AxesHelper(5);
        this.axesHelper.visible = false;
        this.scene.add(this.axesHelper);
    }
    
    // Setup debug overlay
    private setupDebugOverlay(): void {
        this.debugInfoElement = document.createElement('div');
        this.debugInfoElement.style.position = 'absolute';
        this.debugInfoElement.style.top = '10px';
        this.debugInfoElement.style.left = '10px';
        this.debugInfoElement.style.color = 'white';
        this.debugInfoElement.style.backgroundColor = 'rgba(0, 0, 0, 0.5)';
        this.debugInfoElement.style.padding = '5px';
        this.debugInfoElement.style.fontFamily = 'monospace';
        this.debugInfoElement.style.display = 'none'; // Start hidden
        this.canvasContainer.appendChild(this.debugInfoElement);
    }
    
    // Update debug overlay text
    public updateDebugInfo(localPlayer: LocalPlayerRenderData | null, remotePlayers: Map<string, RemotePlayerRenderData>): void {
        if (!this.isDebugActive || !this.debugInfoElement) return;
        
        let debugText = '';
        if (localPlayer) {
            const pos = localPlayer.position;
            debugText += `Local: X:${pos.x.toFixed(2)}, Y:${pos.y.toFixed(2)}, Z:${pos.z.toFixed(2)} `;
            debugText += `(Animation: ${localPlayer.currentAnimationName}, Backwards: ${localPlayer.isMovingBackwards})`;
        } else {
            debugText += 'Local: N/A';
        }

        remotePlayers.forEach((player, id) => {
            // Use player.position.x for X and player.position.y for Z from the DB record
            debugText += `\nRemote ${id.substring(0, 6)}: X:${player.position.x.toFixed(2)}, Z:${player.position.y.toFixed(2)}`;
            debugText += ` (Animation: ${player.animationState})`;
        });

        this.debugInfoElement.textContent = debugText;
    }
    
    // Handle window resize
    private handleResize = (): void => {
        if (!this.canvasContainer || this.isDisposed) return;
        const width = this.canvasContainer.clientWidth;
        const height = this.canvasContainer.clientHeight;
        if (width === 0 || height === 0) return; // Avoid issues if container is hidden

        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(width, height);
    };
    
    // Creates the mutable Three.js components for a player
    public createPlayerRenderComponent(model: THREE.Group): PlayerRenderComponent | null {
        if (!this.areRenderAssetsReady()) {
            console.warn("Attempted to create render component before assets loaded.");
            return null;
        }
        
        // Ensure the top-level model group itself is visible
        model.visible = true;
        
        // Clone materials to ensure each model has its own unique materials
        this.cloneMaterials(model);
        
        let armature: THREE.Object3D | null = null;
        let skinnedMeshFound = false;
        model.traverse((child) => {
            if (child.name === "Armature") {
                armature = child;
            }
            if (child instanceof THREE.SkinnedMesh) {
                skinnedMeshFound = true;
            }
            child.visible = true; // Force all children visible
        });

        if (!skinnedMeshFound) {
            console.error("No SkinnedMesh found in model hierarchy!");
        }
        if (!armature) {
            console.warn("Armature node not found by name 'Armature'. Animation mixer might target the wrong object. Falling back to root.");
            armature = model; // Fallback to root if specific armature isn't found
        }

        // Set up mixers and actions - Target the Armature node if found, otherwise the model root
        const animationTarget = armature;
        const mixer = new THREE.AnimationMixer(animationTarget);

        // Find clips. Use names if available, otherwise indices.
        const idleClip = THREE.AnimationClip.findByName(this.idleClips, 'Idle') || this.idleClips[0];
        // Assume 'Walking' clip is used for both forwards and backwards
        const walkClip = THREE.AnimationClip.findByName(this.walkClips, 'Walking') || this.walkClips[0];

        if (!idleClip) console.error("Idle animation clip not found!");
        if (!walkClip) console.error("Walk animation clip not found!");

        const idleAction = mixer.clipAction(idleClip);
        const walkAction = mixer.clipAction(walkClip);

        // Start with idle playing, walk faded out
        idleAction.setLoop(THREE.LoopRepeat, Infinity).setEffectiveWeight(1).play();
        walkAction.setLoop(THREE.LoopRepeat, Infinity).setEffectiveWeight(0).play(); // Start walk faded out
        walkAction.timeScale = 1; // Default timescale

        return {
            model, // This is the parent Group we transform
            mixer,
            actions: { idle: idleAction, walk: walkAction },
            currentAnimation: 'idle', // Initial client animation *state* is idle
            lastPositionSq: new THREE.Vector2(model.position.x, model.position.z),
        };
    }
    
    // Helper method to clone materials for a model to ensure unique instances
    private cloneMaterials(model: THREE.Group): void {
        model.traverse((node) => {
            if (node instanceof THREE.Mesh) {
                if (Array.isArray(node.material)) {
                    // Handle array of materials
                    node.material = node.material.map(mat => mat.clone());
                } else {
                    // Handle single material
                    node.material = node.material.clone();
                }
            }
        });
    }
    
    // Calculate camera position and look-at target based on player state
    public updateCameraForPlayer(player: LocalPlayerRenderData): void {
        const { position, rotationY, pitch } = player;

        // Base camera position on player
        const camPos = position.clone();
        camPos.y += CAMERA_HEIGHT;

        // Calculate offset based on yaw - increase distance for more distinct separation
        const cameraOffset = new THREE.Vector3(0, 0, CAMERA_DISTANCE * 1.5);
        cameraOffset.applyAxisAngle(new THREE.Vector3(0, 1, 0), rotationY);
        camPos.add(cameraOffset);

        // Adjust height based on pitch (simple vertical offset)
        const verticalOffset = Math.sin(pitch - Math.PI / 2) * CAMERA_DISTANCE;
        camPos.y += verticalOffset;

        // Look at a point slightly above the player's base position
        const lookAtPos = new THREE.Vector3(
            position.x,
            position.y + 1.0, // Look slightly above feet/center mass
            position.z
        );

        this.camera.position.copy(camPos);
        this.camera.lookAt(lookAtPos);
    }
    
    // Add debug markers to a model (arrow pointing forward)
    public addDebugMarkersToModel(model: THREE.Group): void {
        // Add direction indicator (arrow pointing forward)
        const arrowGeometry = new THREE.ConeGeometry(0.2, 0.5, 8);
        const arrowMaterial = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
        const arrow = new THREE.Mesh(arrowGeometry, arrowMaterial);
        arrow.position.set(0, 0, 0);
        arrow.rotation.x = Math.PI / 2; // Point forward (along negative Z)
        arrow.name = "debug_marker_arrow";
        arrow.visible = this.isDebugActive;
        model.add(arrow);
    }
    
    // Toggle debug visualization mode
    public setDebugMode(isActive: boolean): void {
        if (this.isDebugActive === isActive) return;
        
        this.isDebugActive = isActive;
        
        // Toggle scene helpers
        if (this.gridHelper) {
            this.gridHelper.visible = this.isDebugActive;
        }
        if (this.axesHelper) {
            this.axesHelper.visible = this.isDebugActive;
        }
        
        // Toggle player markers
        this.toggleDebugMarkersOnModel(this.localPlayerRenderComponent?.model ?? null, true);
        this.playerRenderComponents.forEach(rc => {
            this.toggleDebugMarkersOnModel(rc.model, false);
        });
        
        // Toggle debug text overlay
        if (this.debugInfoElement) {
            this.debugInfoElement.style.display = this.isDebugActive ? 'block' : 'none';
            if (!this.isDebugActive) {
                this.debugInfoElement.textContent = '';
            }
        }
    }
    
    private toggleDebugMarkersOnModel(model: THREE.Group | null, isLocal: boolean): void {
        if (!model) return;
        const markerName = "debug_marker_arrow";
        const marker = model.getObjectByName(markerName);
        const label = isLocal ? "Local" : `Remote ${model.name}`;

        if (marker) {
            marker.visible = this.isDebugActive;
        } else if (this.isDebugActive) {
            this.addDebugMarkersToModel(model);
        }
    }
    
    // Update local player in the scene based on render data
    public updateLocalPlayer(playerData: LocalPlayerRenderData | null): void {
        if (!playerData) {
            if (this.localPlayerRenderComponent) {
                this.disposeRenderComponent(this.localPlayerRenderComponent);
                this.localPlayerRenderComponent = null;
                // Reset camera
                this.camera.position.set(0, 1, 10);
                this.camera.lookAt(0, 0, 0);
            }
            return;
        }
        
        // Create component if it doesn't exist
        if (!this.localPlayerRenderComponent && this.areRenderAssetsReady()) {
            // Use SkeletonUtils.clone
            const model = SkeletonUtils.clone(this.characterModelTemplate!) as THREE.Group;
            model.name = `PlayerModel_Local_${playerData.identity.substring(0, 6)}`;
            model.position.copy(playerData.position);
            model.rotation.y = playerData.rotationY + Math.PI;
            this.localPlayerRenderComponent = this.createPlayerRenderComponent(model);

            if (this.localPlayerRenderComponent) {
                this.addDebugMarkersToModel(model);
                this.scene.add(this.localPlayerRenderComponent.model);
                
                // Set initial client animation state in component
                this.localPlayerRenderComponent.currentAnimation = playerData.currentAnimationName;
                
                // Set initial time scale based on initial direction
                if (playerData.currentAnimationName === 'walk') {
                    this.localPlayerRenderComponent.actions.walk.timeScale = playerData.isMovingBackwards ? -1 : 1;
                } else {
                    this.localPlayerRenderComponent.actions.walk.timeScale = 1; // Default for idle
                }
                
                // Apply color to local player model if available
                if (playerData.hexColor) {
                    this.applyPlayerColor(model, playerData.hexColor);
                }
            }
        }
        
        // Update existing component
        if (this.localPlayerRenderComponent) {
            const rc = this.localPlayerRenderComponent;
            const model = rc.model;
            
            // Update position and rotation
            model.position.copy(playerData.position);
            model.rotation.y = playerData.rotationY + Math.PI;
            
            // Animation Update: Switch if the animation type changed
            if (rc.currentAnimation !== playerData.currentAnimationName) {
                this.switchAnimation(rc, rc.currentAnimation, playerData.currentAnimationName);
                // Update the component's current animation type *after* switching
                rc.currentAnimation = playerData.currentAnimationName;
            }
            
            // Always update timescale based on current state if walking
            if (playerData.currentAnimationName === 'walk') {
                const targetTimeScale = playerData.isMovingBackwards ? -1 : 1;
                if (rc.actions.walk.timeScale !== targetTimeScale) {
                    rc.actions.walk.timeScale = targetTimeScale;
                }
            } else {
                // Reset timescale when idle
                if (rc.actions.walk.timeScale !== 1) {
                    rc.actions.walk.timeScale = 1;
                }
            }

            // Update color if it has changed
            if (playerData.hexColor) {
                // For existing models, we can make a simple check if it needs color update
                if (!rc.model.userData.currentColor || rc.model.userData.currentColor !== playerData.hexColor) {
                    this.applyPlayerColor(model, playerData.hexColor);
                    rc.model.userData.currentColor = playerData.hexColor; // Track current color
                }
            }
        }
        
        // Update camera position and target based on player
        this.updateCameraForPlayer(playerData);
    }
    
    // Update remote players in the scene based on render data
    public updateRemotePlayers(players: Map<string, RemotePlayerRenderData>): void {
        // Track existing IDs to identify players to remove
        const existingIds = new Set(this.playerRenderComponents.keys());
        
        // Process each player in the update
        players.forEach((playerData, playerId) => {
            existingIds.delete(playerId); // Remove from the set as we process it
            let renderComponent = this.playerRenderComponents.get(playerId);
            
            // Create new component if doesn't exist
            if (!renderComponent && this.areRenderAssetsReady()) {
                const model = SkeletonUtils.clone(this.characterModelTemplate!) as THREE.Group;
                model.name = `PlayerModel_Remote_${playerId.substring(0, 6)}`;
                
                // Set initial position from server data (x, y in server space = x, z in 3D space)
                model.position.set(playerData.position.x, 0, playerData.position.y);
                model.rotation.y = playerData.rotationYaw;
                
                const newComponent = this.createPlayerRenderComponent(model);
                if (newComponent !== null) {
                    // Determine client state and timescale from server state
                    const serverAnimState = playerData.animationState as ServerAnimationState || 'idle';
                    const clientAnimState: AnimationName = (serverAnimState === 'idle') ? 'idle' : 'walk';
                    const targetTimeScale = (serverAnimState === 'walkingBackwards') ? -1 : 1;

                    // Set initial animation state
                    this.switchAnimation(newComponent, 'idle', clientAnimState);
                    newComponent.currentAnimation = clientAnimState;
                    newComponent.actions.walk.timeScale = targetTimeScale;

                    // Apply player color if available
                    if (playerData.hexColor) {
                        this.applyPlayerColor(model, playerData.hexColor);
                    }

                    this.addDebugMarkersToModel(model);
                    this.scene.add(newComponent.model);
                    this.playerRenderComponents.set(playerId, newComponent);
                }
            }
            
            // Update existing component
            if (renderComponent) {
                // Update position and rotation
                const targetPos = new THREE.Vector3(playerData.position.x, 0, playerData.position.y);
                if (!renderComponent.model.position.equals(targetPos)) {
                    renderComponent.model.position.copy(targetPos);
                }
                
                const targetRotY = playerData.rotationYaw;
                if (Math.abs(renderComponent.model.rotation.y - targetRotY) > 0.001) {
                    renderComponent.model.rotation.y = targetRotY;
                }
                
                // Update animation state
                const serverAnimState = (playerData.animationState as ServerAnimationState) || 'idle';
                const desiredClientAnim: AnimationName = (serverAnimState === 'idle') ? 'idle' : 'walk';
                const targetTimeScale = (serverAnimState === 'walkingBackwards') ? -1 : 1;

                // Update animation type if changed
                if (renderComponent.currentAnimation !== desiredClientAnim) {
                    this.switchAnimation(renderComponent, renderComponent.currentAnimation, desiredClientAnim);
                    renderComponent.currentAnimation = desiredClientAnim;
                }

                // Update timescale for walk animation
                if (desiredClientAnim === 'walk') {
                    if (renderComponent.actions.walk.timeScale !== targetTimeScale) {
                        renderComponent.actions.walk.timeScale = targetTimeScale;
                    }
                } else {
                    // Reset timescale when idle
                    if (renderComponent.actions.walk.timeScale !== 1) {
                        renderComponent.actions.walk.timeScale = 1;
                    }
                }
                
                // Update tracking position
                renderComponent.lastPositionSq.set(playerData.position.x, playerData.position.y);
            }
        });
        
        // Remove players that no longer exist
        existingIds.forEach(playerId => {
            const component = this.playerRenderComponents.get(playerId);
            if (component) {
                this.disposeRenderComponent(component);
                this.playerRenderComponents.delete(playerId);
            }
        });
    }
    
    // Update animation mixers with the given delta time
    public updateAnimations(deltaTime: number): void {
        if (this.isDisposed) return;

        // Update local player mixer
        if (this.localPlayerRenderComponent) {
            this.localPlayerRenderComponent.mixer.update(deltaTime);
        }

        // Update other player mixers
        this.playerRenderComponents.forEach(rc => {
            rc.mixer.update(deltaTime);
        });
    }
    
    // Helper function for smooth animation transitions
    public switchAnimation(rc: PlayerRenderComponent, from: AnimationName, to: AnimationName): void {
        if (from === to) return; // No change needed

        // Get the actions for the source and target animations
        const fromAction = rc.actions[from];
        const toAction = rc.actions[to];

        if (!toAction) {
            console.error(`Target animation action for '${to}' could not be determined!`);
            return;
        }
        if (!fromAction) {
            console.warn(`Source animation action for '${from}' could not be determined (might be initial state)`);
        }

        // Smooth transition: Fade out the 'from' action and fade in the 'to' action
        if (fromAction && fromAction !== toAction) { // Check if actions are actually different
            fromAction.fadeOut(ANIMATION_FADE_DURATION);
        }

        // Ensure the target action is reset, faded in, and played
        // Resetting walk ensures it plays from the start when transitioning from idle
        toAction.reset()
               .setEffectiveWeight(1)
               .fadeIn(ANIMATION_FADE_DURATION)
               .play();
    }
    
    // Disposes a render component, cleaning up resources
    private disposeRenderComponent(rc: PlayerRenderComponent | null): void {
        if (!rc) return;
        rc.mixer.stopAllAction(); // Stop animations
        this.disposeModel(rc.model); // Use the general model disposal
    }
    
    // General model disposal
    private disposeModel(model: THREE.Group | null): void {
        if (!model) return;
        if (model.parent) {
            model.removeFromParent();
        } 
        // Recursively dispose of geometry and materials
        model.traverse(child => {
            if (child instanceof THREE.Mesh) {
                child.geometry?.dispose(); 
                const materials = Array.isArray(child.material) ? child.material : [child.material];
                materials.forEach(material => {
                    if (material) {
                        // Dispose textures if they exist
                        material.map?.dispose();
                        material.normalMap?.dispose();
                        material.roughnessMap?.dispose();
                        material.metalnessMap?.dispose();
                        material.emissiveMap?.dispose();
                        material.aoMap?.dispose();
                        material.dispose();
                    }
                });
            }
        });
    }
    
    // Get access to the renderer for the game loop
    public getRenderer(): THREE.WebGLRenderer {
        return this.renderer;
    }
    
    // Render the scene
    public render(): void {
        if (this.isDisposed) return;
        this.renderer.render(this.scene, this.camera);
    }
    
    // Clean up all resources
    public dispose(): void {
        if (this.isDisposed) return;
        this.isDisposed = true;
        
        // Remove event listeners
        window.removeEventListener('resize', this.handleResize);
        
        // Clean up player components
        if (this.localPlayerRenderComponent) {
            this.disposeRenderComponent(this.localPlayerRenderComponent);
            this.localPlayerRenderComponent = null;
        }
        
        this.playerRenderComponents.forEach(rc => {
            this.disposeRenderComponent(rc);
        });
        this.playerRenderComponents.clear();
        
        // Dispose models and assets
        this.disposeModel(this.stadiumModel);
        this.disposeModel(this.characterModelTemplate);
        this.stadiumModel = null;
        this.characterModelTemplate = null;
        
        // Dispose debug helpers
        this.gridHelper?.geometry?.dispose();
        if (this.gridHelper?.material instanceof THREE.Material) {
            this.gridHelper.material.dispose();
        }
        this.axesHelper?.geometry?.dispose();
        if (this.axesHelper?.material instanceof THREE.Material) {
            this.axesHelper.material.dispose();
        }
        
        // Dispose renderer and remove canvas
        if (this.renderer) {
            this.renderer.dispose();
            if (this.renderer.domElement.parentNode === this.canvasContainer) {
                this.canvasContainer.removeChild(this.renderer.domElement);
            }
        }
        
        // Remove debug info element
        if (this.debugInfoElement && this.debugInfoElement.parentNode) {
            this.debugInfoElement.parentNode.removeChild(this.debugInfoElement);
            this.debugInfoElement = null;
        }
        
        // Clear references
        this.scene = null!;
        this.camera = null!;
        this.renderer = null!;
        this.loader = null!;
        this.idleClips = [];
        this.walkClips = [];
    }

    // Apply color to player model
    private applyPlayerColor(model: THREE.Group, hexColor: string): void {
        try {
            // Convert hex to RGB
            const color = new THREE.Color(hexColor);
            
            // Apply to all materials in the model
            model.traverse((node) => {
                if (node instanceof THREE.Mesh && node.material) {
                    // Handle array of materials
                    if (Array.isArray(node.material)) {
                        node.material.forEach(mat => {
                            if (mat.color) {
                                // Keep brightness but change hue
                                const hsl = { h: 0, s: 0, l: 0 };
                                color.getHSL(hsl);
                                const currentHsl = { h: 0, s: 0, l: 0 };
                                mat.color.getHSL(currentHsl);
                                // Preserve original lightness but use new hue
                                mat.color.setHSL(hsl.h, Math.min(1, hsl.s + 0.2), currentHsl.l);
                            }
                        });
                    } 
                    // Handle single material
                    else if (node.material.color) {
                        const hsl = { h: 0, s: 0, l: 0 };
                        color.getHSL(hsl);
                        const currentHsl = { h: 0, s: 0, l: 0 };
                        node.material.color.getHSL(currentHsl);
                        // Preserve original lightness but use new hue
                        node.material.color.setHSL(hsl.h, Math.min(1, hsl.s + 0.2), currentHsl.l);
                    }
                }
            });
            
            // Store the applied color in userData for future reference
            model.userData.currentColor = hexColor;
        } catch (error) {
            console.error(`Failed to apply color ${hexColor} to model:`, error);
        }
    }
}