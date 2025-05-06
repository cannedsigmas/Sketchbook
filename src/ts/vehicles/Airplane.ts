import * as THREE from 'three';
import * as CANNON from 'cannon';

import { Vehicle } from './Vehicle';
import { IControllable } from '../interfaces/IControllable';
import { IWorldEntity } from '../interfaces/IWorldEntity';
import { KeyBinding } from '../core/KeyBinding';
import { SpringSimulator } from '../physics/spring_simulation/SpringSimulator';
import * as Utils from '../core/FunctionLibrary';
import { EntityType } from '../enums/EntityType';

export class Airplane extends Vehicle implements IControllable, IWorldEntity
{
	public entityType: EntityType = EntityType.Airplane;
	public rotor: THREE.Object3D;
	public leftAileron: THREE.Object3D;
	public rightAileron: THREE.Object3D;
	public elevators: THREE.Object3D[] = [];
	public rudder: THREE.Object3D;

	private steeringSimulator: SpringSimulator; 
	private aileronSimulator: SpringSimulator;
	private elevatorSimulator: SpringSimulator;
	private rudderSimulator: SpringSimulator;

	private enginePower: number = 0;
	private lastDrag: number = 0;

	constructor(gltf: any)
	{
		super(gltf, {
			radius: 0.12,
			suspensionStiffness: 150,
			suspensionRestLength: 0.25,
			dampingRelaxation: 5,
			dampingCompression: 5,
			directionLocal: new CANNON.Vec3(0, -1, 0),
			axleLocal: new CANNON.Vec3(-1, 0, 0),
			chassisConnectionPointLocal: new CANNON.Vec3(),
		});

		this.readAirplaneData(gltf);

		this.collision.preStep = (body: CANNON.Body) => { this.physicsPreStep(body, this); };

		this.actions = {
			'throttle': new KeyBinding('ShiftLeft'),
			'brake': new KeyBinding('Space'),
			'wheelBrake': new KeyBinding('KeyB'),
			'pitchUp': new KeyBinding('KeyS'),
			'pitchDown': new KeyBinding('KeyW'),
			'yawLeft': new KeyBinding('KeyQ'),
			'yawRight': new KeyBinding('KeyE'),
			'rollLeft': new KeyBinding('KeyA'),
			'rollRight': new KeyBinding('KeyD'),
			'exitVehicle': new KeyBinding('KeyF'),
			'seat_switch': new KeyBinding('KeyX'),
			'view': new KeyBinding('KeyV'),
		};

		this.steeringSimulator = new SpringSimulator(60, 10, 0.6); 
		this.aileronSimulator = new SpringSimulator(60, 5, 0.6);
		this.elevatorSimulator = new SpringSimulator(60, 7, 0.6);
		this.rudderSimulator = new SpringSimulator(60, 10, 0.6);
	}

	public noDirectionPressed(): boolean
	{
		let result = 
		!this.actions.throttle.isPressed &&
		!this.actions.brake.isPressed &&
		!this.actions.yawLeft.isPressed &&
		!this.actions.yawRight.isPressed &&
		!this.actions.rollLeft.isPressed &&
		!this.actions.rollRight.isPressed;

		return result;
	}

	public update(timeStep: number): void
	{
		super.update(timeStep);
		
		// Rotors visuals
		if (this.controllingCharacter !== undefined)
		{
			// MODIFIED
			if (this.enginePower < 10) this.enginePower += timeStep * 4;
			if (this.enginePower > 10) this.enginePower = 10;
		}
		else
		{
			if (this.enginePower > 0) this.enginePower -= timeStep * 0.12;
			if (this.enginePower < 0) this.enginePower = 0;
		}
		this.rotor.rotateX(this.enginePower * timeStep * 60);

		// Steering logic (unchanged)...

		this.steeringSimulator.simulate(timeStep);
		this.setSteeringValue(this.steeringSimulator.position);

		const partsRotationAmount = 0.7;

		// Ailerons
		if (this.actions.rollLeft.isPressed && !this.actions.rollRight.isPressed)
		{
			this.aileronSimulator.target = partsRotationAmount;
		}
		else if (!this.actions.rollLeft.isPressed && this.actions.rollRight.isPressed)
		{
			this.aileronSimulator.target = -partsRotationAmount;
		}
		else 
		{
			this.aileronSimulator.target = 0;
		}

		// Elevators
		if (this.actions.pitchUp.isPressed && !this.actions.pitchDown.isPressed)
		{
			this.elevatorSimulator.target = partsRotationAmount;
		}
		else if (!this.actions.pitchUp.isPressed && this.actions.pitchDown.isPressed)
		{
			this.elevatorSimulator.target = -partsRotationAmount;
		}
		else
		{
			this.elevatorSimulator.target = 0;
		}

		// Rudder
		if (this.actions.yawLeft.isPressed && !this.actions.yawRight.isPressed)
		{
			this.rudderSimulator.target = partsRotationAmount;
		}
		else if (!this.actions.yawLeft.isPressed && this.actions.yawRight.isPressed)
		{
			this.rudderSimulator.target = -partsRotationAmount;
		}
		else 
		{
			this.rudderSimulator.target = 0;
		}

		// Run rotation simulators
		this.aileronSimulator.simulate(timeStep);
		this.elevatorSimulator.simulate(timeStep);
		this.rudderSimulator.simulate(timeStep);

		// Rotate parts
		this.leftAileron.rotation.y = this.aileronSimulator.position;
		this.rightAileron.rotation.y = -this.aileronSimulator.position;
		this.elevators.forEach((elevator) =>
		{
			elevator.rotation.y = this.elevatorSimulator.position;
		});
		this.rudder.rotation.y = this.rudderSimulator.position;
	}

	public physicsPreStep(body: CANNON.Body, plane: Airplane): void
	{
		let quat = Utils.threeQuat(body.quaternion);
		let right = new THREE.Vector3(1, 0, 0).applyQuaternion(quat);
		let up = new THREE.Vector3(0, 1, 0).applyQuaternion(quat);
		let forward = new THREE.Vector3(0, 0, 1).applyQuaternion(quat);
		
		const velocity = new CANNON.Vec3().copy(this.collision.velocity);
		let velLength1 = body.velocity.length();
		const currentSpeed = velocity.dot(Utils.cannonVector(forward));

		let flightModeInfluence = currentSpeed / 10;
		flightModeInfluence = THREE.MathUtils.clamp(flightModeInfluence, 0, 1);

		let lowerMassInfluence = currentSpeed / 10;
		lowerMassInfluence = THREE.MathUtils.clamp(lowerMassInfluence, 0, 1);
		this.collision.mass = 50 * (1 - (lowerMassInfluence * 0.6));

		// Stabilization logic...

		// Control forces (pitch/yaw/roll)...

		// Thrust
		let speedModifier = 0.02;
		if (plane.actions.throttle.isPressed && !plane.actions.brake.isPressed)
		{
			speedModifier = 0.6; // MODIFIED
		}
		else if (!plane.actions.throttle.isPressed && plane.actions.brake.isPressed)
		{
			speedModifier = -0.05;
		}
		else if (this.rayCastVehicle.numWheelsOnGround > 0)
		{
			speedModifier = 0;
		}

		body.velocity.x += (velLength1 * this.lastDrag + speedModifier) * forward.x * this.enginePower;
		body.velocity.y += (velLength1 * this.lastDrag + speedModifier) * forward.y * this.enginePower;
		body.velocity.z += (velLength1 * this.lastDrag + speedModifier) * forward.z * this.enginePower;

		// Drag
		let velLength2 = body.velocity.length();
		const drag = Math.pow(velLength2, 1) * 0.001 * this.enginePower; // MODIFIED
		body.velocity.x -= body.velocity.x * drag;
		body.velocity.y -= body.velocity.y * drag;
		body.velocity.z -= body.velocity.z * drag;
		this.lastDrag = drag;

		// Lift
		let lift = Math.pow(velLength2, 1) * 0.005 * this.enginePower;
		lift = THREE.MathUtils.clamp(lift, 0, 0.05);
		body.velocity.x += up.x * lift;
		body.velocity.y += up.y * lift;
		body.velocity.z += up.z * lift;

		// Angular damping...
		body.angularVelocity.x = THREE.MathUtils.lerp(body.angularVelocity.x, body.angularVelocity.x * 0.98, flightModeInfluence);
		body.angularVelocity.y = THREE.MathUtils.lerp(body.angularVelocity.y, body.angularVelocity.y * 0.98, flightModeInfluence);
		body.angularVelocity.z = THREE.MathUtils.lerp(body.angularVelocity.z, body.angularVelocity.z * 0.98, flightModeInfluence);
	}

	// Remaining methods (input, readAirplaneData, etc.) are unchanged
	public onInputChange(): void { /* ... */ }
	public readAirplaneData(gltf: any): void { /* ... */ }
	public inputReceiverInit(): void { /* ... */ }
}
