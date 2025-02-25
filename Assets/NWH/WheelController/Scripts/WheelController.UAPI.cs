using UnityEngine;
using NWH.Common.Vehicles;


namespace NWH.WheelController3D
{
    public partial class WheelController
    {
        [ShowInTelemetry]
        public override float MotorTorque
        {
            get => motorTorque;
            set => motorTorque = value;
        }

        [ShowInTelemetry]
        public override float CounterTorque
        {
            get => counterTorque;
        }

        [ShowInTelemetry]
        public override float BrakeTorque
        {
            get => brakeTorque;
            set => brakeTorque = value;
        }

        [ShowInTelemetry]
        public override float SteerAngle
        {
            get => steerAngle;
            set => steerAngle = value;
        }

        public override float Mass
        {
            get => wheel.mass;
            set
            {
                wheel.mass = Mathf.Clamp(value, 0f, Mathf.Infinity);
            }
        }

        public override float Radius
        {
            get => wheel.radius;
            set
            {
                wheel.radius = value < Mathf.Epsilon ? Mathf.Epsilon : value;
                wheel.invRadius = 1f / wheel.radius;
            }
        }

        public override float Width
        {
            get => wheel.width;
            set
            {
                wheel.width = value;
            }
        }

        public override float Inertia
        {
            get => wheel.inertia;
            set
            {
                wheel.inertia = value < Mathf.Epsilon ? Mathf.Epsilon : value;
                wheel.invInertia = 1f / wheel.inertia;
            }
        }


        [ShowInTelemetry]
        public override float RPM
        {
            get => wheel.rpm;
        }

        public override float AngularVelocity
        {
            get => wheel.angularVelocity;
        }

        public override Vector3 WheelPosition
        {
            get => _wheelWorldPosition;
        }

        [ShowInTelemetry]
        public override float Load
        {
            get => load;
        }

        public override float MaxLoad
        {
            get => loadRating;
            set => loadRating = value < 0f ? 0f : value;
        }

        [ShowInTelemetry]
        public override float Camber
        {
            get => camber;
            set => camber = value < -16f ? -16f : value > 16f ? 16f : value;
        }

        [ShowInTelemetry]
        public override bool IsGrounded
        {
            get => _isGrounded;
        }

        [ShowInTelemetry]
        public override float Damage
        {
            get => _damage;
            set => _damage = value < 0f ? 0f : value > 1f ? 1f : value;
        }

        public override float SpringMaxLength
        {
            get => spring.maxLength;
            set
            {
                spring.maxLength = value < 0f ? 0f : value;
            }
        }

        public override float SpringMaxForce
        {
            get => spring.maxForce;
            set
            {
                spring.maxForce = value < 0f ? 0f : value;
            }
        }

        [ShowInTelemetry]
        public override float SpringForce
        {
            get => spring.force;
        }

        [ShowInTelemetry]
        public override float SpringLength
        {
            get => spring.length;
        }

        public override float SpringCompression
        {
            get => spring.maxLength == 0f ? 1f
                : (spring.maxLength - spring.length) / spring.maxLength;
        }

        public override float DamperBumpRate
        {
            get => damper.bumpRate;
            set => damper.bumpRate = value < 0f ? 0f : value;
        }

        public override float DamperReboundRate
        {
            get => damper.reboundRate;
            set => damper.reboundRate = value < 0f ? 0f : value;
        } // TODO - finish this

        [ShowInTelemetry]
        public override float DamperForce
        {
            get => damper.force;
        }

        [ShowInTelemetry]
        public override float LongitudinalSlip
        {
            get => forwardFriction.slip;
        }

        [ShowInTelemetry]
        public override float LongitudinalSpeed
        {
            get => forwardFriction.speed;
        }

        [ShowInTelemetry]
        public override float LateralSlip
        {
            get => sideFriction.slip;
        }

        [ShowInTelemetry]
        public override float LateralSpeed
        {
            get => sideFriction.speed;
        }

        public override Vector3 HitPoint
        {
            get
            {
                return wheelHit.point;
            }
        }

        public override Vector3 HitNormal
        {
            get
            {
                return wheelHit.normal;
            }
        }

        public override GameObject WheelVisual
        {
            get => wheel.rotatingContainer.gameObject;
            set
            {
                wheel.rotatingContainer = value.transform;
            }
        }

        public override GameObject NonRotatingVisual
        {
            get => wheel.nonRotatingContainer.gameObject;
            set
            {
                wheel.nonRotatingContainer = value.transform;
            }
        }

        public override Rigidbody TargetRigidbody
        {
            get => targetRigidbody;
        }

        public override Collider HitCollider
        {
            get => wheelHit.collider;
        }

        public override float ForceApplicationPointDistance
        {
            get => forceApplicationPointDistance;
            set => forceApplicationPointDistance = value;
        }

        public override FrictionPreset FrictionPreset
        {
            get => activeFrictionPreset;
            set => activeFrictionPreset = value;
        }

        public override float LongitudinalFrictionGrip
        {
            get => forwardFriction.grip;
            set => forwardFriction.grip = value < 0f ? 0f : value;
        }

        public override float LongitudinalFrictionStiffness
        {
            get => forwardFriction.stiffness;
            set => forwardFriction.stiffness = value < 0f ? 0f : value;
        }

        public override float LateralFrictionGrip
        {
            get => sideFriction.grip;
            set => sideFriction.grip = value < 0f ? 0f : value;
        }

        public override float LateralFrictionStiffness
        {
            get => sideFriction.stiffness;
            set => sideFriction.stiffness = value < 0f ? 0f : value;
        }

        public override float RollingResistanceTorque
        {
            get => rollingResistanceTorque;
            set => rollingResistanceTorque = value < 0f ? 0f : value;
        }

        public override float FrictionCircleShape
        {
            get => frictionCirclePower;
            set => frictionCirclePower = value < 0f ? 0f : value;
        }

        public override float FrictionCircleStrength
        {
            get => frictionCircleStrength;
            set => frictionCircleStrength = value < 0f ? 0f : value;
        }

        public override bool AutoSimulate
        {
            get => _autoSimulate;
            set => _autoSimulate = value;
        }

        public override void Validate()
        {
            OnValidate();
        }

        public override void WakeFromSleep()
        {
            _wakeOneFrame = true;
        }
    }
}
