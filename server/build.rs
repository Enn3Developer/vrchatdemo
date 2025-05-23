use gltf::buffer::Data;
use gltf::Node;
use nalgebra::{ArrayStorage, Matrix4, Point3, Transform3};
use std::path::PathBuf;

const MODEL_PATH: &'static str = "../client/public/models/forest_scene/scene.glb";
const INITIAL_SCALING: f32 = 1.0;

trait ConvertData<V, I> {
    fn convert_data(self) -> Vec<ColliderData<V, I>>;
}

pub trait ToBytes {
    fn to_bytes(&self) -> Vec<u8>;
}

pub struct ColliderData<V, I> {
    pub vertices: Vec<V>,
    pub indices: Vec<I>,
}

type VectorTuple = (f32, f32, f32);
type VectorArray<T> = [T; 3];

type AlgebraColliderData = ColliderData<Point3<f32>, VectorArray<u32>>;
type NormalColliderData = ColliderData<VectorTuple, VectorArray<u32>>;

impl AlgebraColliderData {
    fn generate_collider_data(
        node: Node,
        transform: Transform3<f32>,
        buffers: &Vec<Data>,
    ) -> Vec<Self> {
        // get the current node's transformation
        let node_transform: Transform3<f32> = Transform3::from_matrix_unchecked(
            Matrix4::<f32>::from_data(ArrayStorage(node.transform().matrix())),
        );

        // compute the overall transformation of current node * parent node
        let transform: Transform3<f32> = node_transform * transform;

        let mut collider_data = vec![];

        // for every node's child
        for child in node.children() {
            // generate the collider for that child
            for collider in Self::generate_collider_data(child, transform, buffers) {
                collider_data.push(collider);
            }
        }

        // if this node has a mesh
        if let Some(mesh) = node.mesh() {
            // setup vertex data for the trimesh
            let mut vertices = vec![];
            let mut indices = vec![];

            // for every primitive in the mesh
            for primitive in mesh.primitives() {
                // set up the reader
                let reader = primitive.reader(|buffer| Some(&buffers[buffer.index()]));

                // read all vertex positions
                if let Some(positions) = reader.read_positions() {
                    for vertex_position in positions {
                        // apply the node transformation to the vertex
                        let vertex = transform
                            * Point3::<f32>::new(
                                vertex_position[0],
                                vertex_position[1],
                                vertex_position[2],
                            );

                        vertices.push(vertex);
                    }
                }

                // read all indices for the faces of the mesh
                if let Some(primitive_indices) = reader.read_indices() {
                    // set up data
                    let mut vector: [u32; 3] = [0, 0, 0];
                    let mut i = 0;

                    // for every index
                    for index in primitive_indices.into_u32() {
                        // save the index in an array
                        vector[i] = index;
                        i += 1;

                        // if a triangle is formed
                        if i >= 3 {
                            // save the indices of the triangle
                            indices.push(vector);
                            // and reset the counter
                            i = 0;
                        }
                    }
                }
            }

            collider_data.push(ColliderData { vertices, indices });
        }

        collider_data
    }
}

impl ConvertData<VectorTuple, VectorArray<u32>> for Vec<AlgebraColliderData> {
    fn convert_data(self) -> Vec<NormalColliderData> {
        self.into_iter()
            .map(|data| ColliderData {
                indices: data.indices,
                vertices: data
                    .vertices
                    .iter()
                    .map(|vertex| (vertex.x, vertex.y, vertex.z))
                    .collect(),
            })
            .collect()
    }
}

impl ToBytes for NormalColliderData {
    fn to_bytes(&self) -> Vec<u8> {
        let mut bytes = vec![];

        let mut indices_bytes = self.indices.to_bytes();
        let mut vertices_bytes = self.vertices.to_bytes();

        bytes.append(&mut indices_bytes);
        bytes.append(&mut vertices_bytes);

        bytes
    }
}

impl<T: ToBytes> ToBytes for Vec<T> {
    fn to_bytes(&self) -> Vec<u8> {
        let mut bytes = vec![];

        let mut len = (self.len() as u64).to_le_bytes().to_vec();
        bytes.append(&mut len);
        for element in self {
            let mut element_bytes = element.to_bytes();
            bytes.append(&mut element_bytes);
        }

        bytes
    }
}

impl ToBytes for [u32; 3] {
    fn to_bytes(&self) -> Vec<u8> {
        let mut bytes = vec![];

        for i in 0..3 {
            let mut element_bytes = self[i].to_le_bytes().to_vec();
            bytes.append(&mut element_bytes);
        }

        bytes
    }
}

impl ToBytes for (f32, f32, f32) {
    fn to_bytes(&self) -> Vec<u8> {
        let mut bytes = vec![];

        let mut element_bytes = self.0.to_le_bytes().to_vec();
        bytes.append(&mut element_bytes);

        let mut element_bytes = self.1.to_le_bytes().to_vec();
        bytes.append(&mut element_bytes);

        let mut element_bytes = self.2.to_le_bytes().to_vec();
        bytes.append(&mut element_bytes);

        bytes
    }
}

fn main() {
    println!("cargo:rerun-if-changed={MODEL_PATH}");

    // import model
    let (document, buffers, _) = gltf::import(MODEL_PATH).expect("can't import model");

    let mut collider_data = vec![];

    // generate colliders data
    for scene in document.scenes() {
        for node in scene.nodes() {
            let identity: Transform3<f32> = Transform3::from_matrix_unchecked(
                Matrix4::<f32>::identity().scale(INITIAL_SCALING),
            );

            for collider in
                ColliderData::generate_collider_data(node, identity, &buffers).convert_data()
            {
                collider_data.push(collider);
            }
        }
    }

    // write content of the collider data file
    let content = collider_data.to_bytes();

    // file_path: $OUT_DIR/collider_data.rs
    let mut file_path = PathBuf::new();
    file_path.push(&std::env::var_os("OUT_DIR").unwrap());
    file_path.push("collider_data");

    std::fs::write(file_path, content).expect("can't write to output file");
}
