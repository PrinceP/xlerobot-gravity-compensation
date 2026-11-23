//! Gravity model of one SO-101 arm from the XLeRobot URDF.
//!
//! The arm subtree (from `Base` / `Base_2`) is loaded into a small tree of links.
//! Holding torque for joint i (what Pinocchio's `computeGeneralizedGravity` returns):
//!
//!   g_i(q) = - a_i · Σ_{links j below joint i} (c_j - p_i) × (m_j · g)
//!
//! with a_i, p_i the joint axis and origin in the world (cart) frame, c_j the link
//! centre of mass and g = (0, 0, -9.81). This is the gravity part of RNEA for a tree.

use std::collections::HashMap;
use std::path::Path;

use anyhow::{Context, Result, bail};
use nalgebra::{Isometry3, Point3, Translation3, Unit, UnitQuaternion, Vector3};

pub const GRAVITY: Vector3<f64> = Vector3::new(0.0, 0.0, -9.81);

#[derive(Debug, Clone)]
struct Node {
    parent: Option<usize>,
    /// Fixed transform parent link -> this link at q = 0 (the joint origin).
    origin: Isometry3<f64>,
    axis: Unit<Vector3<f64>>,
    /// Index into the actuated-joint vector, or None for a fixed joint.
    q_index: Option<usize>,
    mass: f64,
    com: Point3<f64>,
}

#[derive(Debug, Clone)]
pub struct ArmModel {
    nodes: Vec<Node>, // topological order: parent before child
    pub joint_names: Vec<String>,
}

fn iso(xyz: [f64; 3], rpy: [f64; 3]) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::new(xyz[0], xyz[1], xyz[2]),
        UnitQuaternion::from_euler_angles(rpy[0], rpy[1], rpy[2]),
    )
}

/// An extra point mass on a link (wrist camera, payload), in that link's frame.
#[derive(Debug, Clone, serde::Deserialize)]
pub struct ExtraMass {
    pub link: String,
    pub mass_kg: f64,
    pub xyz: [f64; 3],
}

impl ArmModel {
    /// Load the subtree rooted at `root_link`. Everything above it (cart, mounts) is
    /// fixed, so its transform to `world_link` is folded into the root origin.
    /// `joints` names the actuated joints in the order the q vector uses.
    pub fn from_urdf(
        path: &Path,
        world_link: &str,
        root_link: &str,
        joints: &[String],
        extra: &[ExtraMass],
    ) -> Result<Self> {
        let robot = urdf_rs::read_file(path).with_context(|| format!("read URDF {}", path.display()))?;
        let link_by_name: HashMap<&str, &urdf_rs::Link> =
            robot.links.iter().map(|l| (l.name.as_str(), l)).collect();
        let joint_to_child: HashMap<&str, &urdf_rs::Joint> =
            robot.joints.iter().map(|j| (j.child.link.as_str(), j)).collect();

        // Fixed chain world_link -> root_link.
        let mut root_pose = Isometry3::identity();
        let mut cur = root_link;
        while cur != world_link {
            let j = joint_to_child
                .get(cur)
                .with_context(|| format!("no path from {world_link} to {root_link}"))?;
            if !matches!(j.joint_type, urdf_rs::JointType::Fixed) {
                bail!("joint {} between {world_link} and {root_link} is not fixed", j.name);
            }
            root_pose = iso(j.origin.xyz.0, j.origin.rpy.0) * root_pose;
            cur = j.parent.link.as_str();
        }

        let mut nodes = Vec::new();
        let mut index_of: HashMap<String, usize> = HashMap::new();
        let mut stack = vec![(root_link.to_string(), None::<usize>, root_pose, None::<&urdf_rs::Joint>)];
        let mut seen_joints = vec![false; joints.len()];
        while let Some((name, parent, origin, joint)) = stack.pop() {
            let link = link_by_name.get(name.as_str()).with_context(|| format!("link {name}"))?;
            let (axis, q_index) = match joint {
                Some(j) if !matches!(j.joint_type, urdf_rs::JointType::Fixed) => {
                    let qi = joints.iter().position(|n| *n == j.name);
                    if let Some(i) = qi {
                        seen_joints[i] = true;
                    }
                    let a = j.axis.xyz.0;
                    (Unit::new_normalize(Vector3::new(a[0], a[1], a[2])), qi)
                }
                _ => (Vector3::z_axis(), None),
            };
            let c = link.inertial.origin.xyz.0;
            let idx = nodes.len();
            nodes.push(Node {
                parent,
                origin,
                axis,
                q_index,
                mass: link.inertial.mass.value,
                com: Point3::new(c[0], c[1], c[2]),
            });
            index_of.insert(name.clone(), idx);
            for j in robot.joints.iter().filter(|j| j.parent.link == name) {
                stack.push((j.child.link.clone(), Some(idx), iso(j.origin.xyz.0, j.origin.rpy.0), Some(j)));
            }
        }
        if let Some(i) = seen_joints.iter().position(|s| !s) {
            bail!("joint {} not found below {root_link}", joints[i]);
        }

        // Extra point masses: merge into the link's mass and CoM.
        for e in extra {
            let &i = index_of
                .get(&e.link)
                .with_context(|| format!("extra mass link {} not in arm", e.link))?;
            let n = &mut nodes[i];
            let p = Point3::new(e.xyz[0], e.xyz[1], e.xyz[2]);
            let m = n.mass + e.mass_kg;
            if m > 0.0 {
                n.com = Point3::from((n.com.coords * n.mass + p.coords * e.mass_kg) / m);
            }
            n.mass = m;
        }

        Ok(Self { nodes, joint_names: joints.to_vec() })
    }

    pub fn total_mass(&self) -> f64 {
        self.nodes.iter().map(|n| n.mass).sum()
    }

    /// Holding torque (N·m) per actuated joint, URDF sign convention.
    pub fn gravity(&self, q: &[f64]) -> Vec<f64> {
        assert_eq!(q.len(), self.joint_names.len());
        let n = self.nodes.len();
        let mut pose = vec![Isometry3::identity(); n];
        let mut tau = vec![0.0; q.len()];
        // Joint origin and world axis for each actuated node.
        let mut joint_frame: Vec<Option<(Point3<f64>, Vector3<f64>)>> = vec![None; n];
        for (i, node) in self.nodes.iter().enumerate() {
            let parent = node.parent.map_or(Isometry3::identity(), |p| pose[p]);
            let at_origin = parent * node.origin;
            let qv = node.q_index.map_or(0.0, |k| q[k]);
            pose[i] = at_origin * UnitQuaternion::from_axis_angle(&node.axis, qv);
            if node.q_index.is_some() {
                joint_frame[i] = Some((at_origin * Point3::origin(), at_origin.rotation * node.axis.into_inner()));
            }
        }
        for (j, node) in self.nodes.iter().enumerate() {
            if node.mass <= 0.0 {
                continue;
            }
            let c = pose[j] * node.com;
            let f = GRAVITY * node.mass;
            let mut cur = Some(j);
            while let Some(i) = cur {
                if let (Some(k), Some((p, a))) = (self.nodes[i].q_index, joint_frame[i]) {
                    tau[k] -= a.dot(&(c - p).cross(&f));
                }
                cur = self.nodes[i].parent;
            }
        }
        tau
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn arm() -> ArmModel {
        let names: Vec<String> = ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll", "Jaw"]
            .iter()
            .map(|s| s.to_string())
            .collect();
        ArmModel::from_urdf(Path::new("urdf/xlerobot.urdf"), "base_link", "Base", &names, &[]).unwrap()
    }

    #[test]
    fn loads_arm() {
        let m = arm();
        assert!((m.total_mass() - 0.801).abs() < 0.01, "mass {}", m.total_mass());
    }

    /// Finite-difference check: g(q) = dV/dq, V = potential energy.
    #[test]
    fn gravity_is_gradient_of_potential() {
        let m = arm();
        let potential = |q: &[f64]| -> f64 {
            // Re-derive CoM heights through the public API by integrating is circular;
            // instead compute V directly here.
            let n = m.nodes.len();
            let mut pose = vec![Isometry3::identity(); n];
            let mut v = 0.0;
            for (i, node) in m.nodes.iter().enumerate() {
                let parent = node.parent.map_or(Isometry3::identity(), |p| pose[p]);
                let qv = node.q_index.map_or(0.0, |k| q[k]);
                pose[i] = parent * node.origin * UnitQuaternion::from_axis_angle(&node.axis, qv);
                v += node.mass * 9.81 * (pose[i] * node.com).z;
            }
            v
        };
        let q = [0.3, 1.2, 1.0, -0.4, 0.7, 0.2];
        let g = m.gravity(&q);
        for k in 0..q.len() {
            let h = 1e-6;
            let mut qp = q;
            let mut qm = q;
            qp[k] += h;
            qm[k] -= h;
            let fd = (potential(&qp) - potential(&qm)) / (2.0 * h);
            assert!((fd - g[k]).abs() < 1e-6, "joint {k}: fd {fd} vs {}", g[k]);
        }
    }
}
