#include <poisson-recon/Reconstruction.h>

#include <Open3D/Open3D.h>

#include <vector>
#include <memory>

void meshIt(open3d::geometry::PointCloud& pcd, open3d::geometry::TriangleMesh& mesh, double density_threshold = 0.0, size_t depth = 10, double weight = 5, size_t width = 0, double scale = 1.1, bool linear_fit = false)
{
  std::cout << "Meshing..." << std::endl;
  path_poisson::TriangleMesh mesh_tmp;
  path_poisson::MeshOptions mesh_ops;
  mesh_ops.verbose = false;
  path_poisson::PointCloud input_poisson;
  input_poisson.points_ = pcd.points_;
  input_poisson.normals_ = pcd.normals_;
  auto tuple_tmp = path_poisson::createPoissonMeshFromPointCloud(input_poisson, depth, weight, width, scale, linear_fit, mesh_ops);
  //std::cout<<tuple_tmp;
  mesh_tmp = std::get<0>(tuple_tmp);
  mesh.adjacency_list_ = mesh_tmp.adjacency_list_;
  mesh.vertices_ = mesh_tmp.vertices_;
  mesh.vertex_normals_ = mesh_tmp.vertex_normals_;
  mesh.triangles_ = mesh_tmp.triangles_;
  mesh.triangle_normals_ = mesh_tmp.triangle_normals_;

  mesh = mesh.ComputeTriangleNormals(false);
  // mesh = SimplifyQuadricDecimation(mesh,10000);

  open3d::visualization::DrawGeometries({ std::make_shared<open3d::geometry::TriangleMesh>(mesh) });

  if (density_threshold > 0)
  {
    std::cout << "Simplifying mesh based on density..." << std::endl;
    auto density = std::get<1>(tuple_tmp);
    std::vector<bool> mask(density.size(), false);
    for (auto k1 = 0; k1 < density.size(); k1++)
    {
      mask[k1] = density[k1] < density_threshold;
    }
    mesh.RemoveVerticesByMask(mask);
    mesh.RemoveUnreferencedVertices();
  }

  std::cout << "Done..." << std::endl;
}

bool seam_scan_inspection(std::shared_ptr<open3d::geometry::KDTreeFlann>& scan_tree,
                          std::shared_ptr<open3d::geometry::KDTreeFlann>& cad_tree,
                          const open3d::geometry::PointCloud& seam,
                          double scoop_radius,
                          double scan_density_threshold)
{
  std::cout << "Seam scan inspection" << std::endl;
  // things to keep track of
  double num_cad_points = 1;
  double num_scan_points = 1;
  int num_valid_boxes = 0;

  // evaluate along the seam
  for (auto point : seam.points_)
  {
    // count number of points in the scan
    std::vector<int> scan_tmp_inds;
    std::vector<double> scan_tmp_dists;
    scan_tree->SearchRadius(point, scoop_radius, scan_tmp_inds, scan_tmp_dists);
    double this_scan_points = (double)scan_tmp_inds.size();

    // count number of points in the CAD model
    std::vector<int> cad_tmp_inds;
    std::vector<double> cad_tmp_dists;
    cad_tree->SearchRadius(point, scoop_radius, cad_tmp_inds, cad_tmp_dists);
    double this_cad_points = (double)cad_tmp_inds.size();

    // check if there are enough points in the scan compared to CAD model
    if (this_scan_points / this_cad_points > scan_density_threshold)
    {
      num_valid_boxes += 1;
    }
    //        std::cout << this_scan_points << "-" << this_cad_points << " | ";
    num_scan_points += this_scan_points;
    num_cad_points += this_cad_points;
  }
  //    std::cout << std::endl;

  std::cout << "\tData: num_valid_boxes = " << num_valid_boxes << std::endl;
  std::cout << "\tData: total_num_boxes = " << seam.points_.size() << std::endl;
  std::cout << "\tData: num_scan_points = " << num_scan_points << std::endl;
  std::cout << "\tData: num_cad_points = " << num_cad_points << std::endl;
  std::cout << "\tData: num_scan_points/num_cad_points = " << num_scan_points / num_cad_points << std::endl;

  // make final decision
  bool condition_1 = true;  //num_valid_boxes==seam.points_.size();
  bool condition_2 = (num_scan_points / num_cad_points) > scan_density_threshold;
  std::cout << "\tResults: " << condition_1 << " and " << condition_2 << std::endl;
  return not(condition_1 && condition_2);
}

void hybridizeSeamAreas(open3d::geometry::PointCloud& scan,
                        open3d::geometry::PointCloud& transformed_model,
                        std::shared_ptr<open3d::geometry::KDTreeFlann> scan_tree,
                        std::shared_ptr<open3d::geometry::KDTreeFlann> cad_tree,
                        const std::vector<open3d::geometry::PointCloud>& transformed_seams,
                        open3d::geometry::PointCloud& hybrid_pcd,
                        bool replace_all_seams,
                        double scoop_radius_,
                        double scan_density_threshold)
{
  // to keep track of inds -- add all scan points and don't add any cad points by default
  std::vector<int> scan_inds_to_add(scan.points_.size(), 1);
  std::vector<int> cad_inds_to_add(transformed_model.points_.size(), 0);

  // process each seam
  std::cout << "Processing " << transformed_seams.size() << " transformed_seams\n";
  //#pragma omp parallel for schedule(static) default(none) shared(transformed_seams, replace_all_seams, scoop_radius_, scan_density_threshold, scan_tree, cad_tree, scan_inds_to_add, cad_inds_to_add)
  for (size_t si = 0; si < transformed_seams.size(); ++si)
  {
    std::cout << "Processing seam " << si << "/" << transformed_seams.size() << std::endl;
    const auto& seam = transformed_seams[si];

    if (replace_all_seams || seam_scan_inspection(scan_tree, cad_tree, seam, scoop_radius_, scan_density_threshold))
    {
      // each point in each seam
      for (const auto& point : seam.points_)
      {
        // query ball from scan around each point in seam and collect inds
        std::vector<int> scan_tmp_inds;
        std::vector<double> scan_tmp_dists;
        scan_tree->SearchRadius(point, scoop_radius_, scan_tmp_inds, scan_tmp_dists);
        for (auto ind : scan_tmp_inds)
        {
          scan_inds_to_add[ind] = 0;
        }

        // query ball from transformed_model around seam and collect inds
        std::vector<int> cad_tmp_inds;
        std::vector<double> cad_tmp_dists;
        cad_tree->SearchRadius(point, scoop_radius_, cad_tmp_inds, cad_tmp_dists);
        for (auto ind : cad_tmp_inds)
        {
          cad_inds_to_add[ind] = 1;
        }
      }
    }
  }

  // make hybrid pcd
  printf("Making hybrid point cloud from model and scan...\n");
  int count = 0;

  // add points from scan
  for (size_t ind = 0; ind < scan.points_.size(); ++ind)
  {
    // inverting inds to ignore points around seam
    if (scan_inds_to_add[ind] == 1)
    {
      count++;
      hybrid_pcd.points_.push_back(scan.points_[ind]);
      hybrid_pcd.normals_.push_back(scan.normals_[ind]);
      hybrid_pcd.colors_.emplace_back(1.0, 0.5, 0.5);
    }
  }
  std::cout << "\tTaking " << count << " points from the scan\n";

  // add points from model
  count = 0;
  for (size_t ind = 0; ind < transformed_model.points_.size(); ++ind)
  {
    // inverting inds to ignore points around seam
    if (cad_inds_to_add[ind] == 1)
    {
      count++;
      hybrid_pcd.points_.push_back(transformed_model.points_[ind]);
      hybrid_pcd.normals_.push_back(transformed_model.normals_[ind]);
      hybrid_pcd.colors_.emplace_back(0.5, 0.5, 1.);
    }
  }
  std::cout << "\tTaking " << count << " points from the model\n";
  std::cout << "Done\n";
}

void hybridizeNonSeamAreas(open3d::geometry::PointCloud& scan,
                           open3d::geometry::PointCloud& transformed_model,
                           std::shared_ptr<open3d::geometry::KDTreeFlann> scan_tree,
                           open3d::geometry::PointCloud& hybrid_pcd,
                           double distance_threshold = 0.0001)
{
  std::vector<bool> cad_to_hybrid_mask(transformed_model.points_.size(), false);
  // for each point in CAD model
#pragma omp parallel for schedule(static) default(none) shared(scan, transformed_model, scan_tree, hybrid_pcd, distance_threshold, cad_to_hybrid_mask)
  for (size_t i = 0; i < transformed_model.points_.size(); i++)
  {
    // find nearest point in scan
    auto point = transformed_model.points_[i];
    std::vector<int> scan_tmp_inds;
    std::vector<double> scan_tmp_dists;
    scan_tree->SearchKNN(point, 1, scan_tmp_inds, scan_tmp_dists);

    // add model point to hybrid pcd if its too far away
    if (scan_tmp_dists[0] > distance_threshold)
    {
      cad_to_hybrid_mask[i] = true;
    }
  }

  for (size_t ind = 0; ind < cad_to_hybrid_mask.size(); ind++)
  {
    if (cad_to_hybrid_mask[ind])
    {
      hybrid_pcd.points_.push_back(transformed_model.points_[ind]);
      hybrid_pcd.normals_.push_back(transformed_model.normals_[ind]);
      hybrid_pcd.colors_.emplace_back(0.5, 0.5, 1.);
    }
  }
}

void makeHybridPointCloud(const open3d::geometry::PointCloud& tmp_scan,
                          const open3d::geometry::PointCloud& tmp_transformed_model,
                          const std::vector<open3d::geometry::PointCloud>& transformed_seams,
                          open3d::geometry::PointCloud& hybrid_pcd,
                          open3d::geometry::TriangleMesh& hybrid_mesh,
                          bool replace_all_seams,
                          double scoop_radius_,
                          double scan_density_threshold,
                          bool viz_)
{
  std::cout << replace_all_seams << " | " << scoop_radius_ << " | " << scan_density_threshold << " | " << viz_ << std::endl;
  // voxel down sample
  std::cout << "Before voxel downsampling:: tmp_scan: " << tmp_scan.points_.size() << std::endl;
  printf("Voxel Downsampling\n");
  auto scan = *tmp_scan.VoxelDownSample(0.00075);
  auto transformed_model = *tmp_transformed_model.VoxelDownSample(0.00075);
  std::cout << "After voxel downsampling:: tmp_scan:  " << tmp_scan.points_.size() << std::endl;
  std::cout << "After voxel downsampling:: scan:  " << scan.points_.size() << std::endl;

  // create kd tree for scan if necessary
  auto scan_tree = std::make_shared<open3d::geometry::KDTreeFlann>();
  printf("Creating KDTree for scan...\n");
  scan_tree->SetGeometry(scan);
  printf("Done\n");

  // create kd tree for transformed_model model if necessary
  auto cad_tree = std::make_shared<open3d::geometry::KDTreeFlann>();
  printf("Creating KDTree for model...\n");
  cad_tree->SetGeometry(transformed_model);
  printf("Done\n");

  // Mesh hybrid point cloud and do density based triangle deletion a.k.a bubble popping
  open3d::geometry::TriangleMesh scan_mesh;
  meshIt(scan, scan_mesh, 7.0);
  // show
  if (viz_)
  {
    open3d::visualization::DrawGeometries({ std::make_shared<open3d::geometry::TriangleMesh>(scan_mesh) });
  }

  // replace seam areas using CAD model as required depending on replace_all_seam flag
  hybridizeSeamAreas(scan,
                     transformed_model,
                     scan_tree,
                     cad_tree,
                     transformed_seams,
                     hybrid_pcd,
                     replace_all_seams,
                     scoop_radius_,
                     scan_density_threshold);
  // show
  if (viz_)
  {
    open3d::visualization::DrawGeometries({ std::make_shared<open3d::geometry::PointCloud>(hybrid_pcd) });
  }

  // replace non-seam areas using CAD model if there are holes
  hybridizeNonSeamAreas(scan,
                        transformed_model,
                        scan_tree,
                        hybrid_pcd);
  // show
  if (viz_)
  {
    open3d::visualization::DrawGeometries({ std::make_shared<open3d::geometry::PointCloud>(hybrid_pcd) });
  }

  // Mesh hybrid point cloud and do density based triangle deletion a.k.a bubble popping
  meshIt(hybrid_pcd, hybrid_mesh, 7.0);

  // show
  if (viz_)
  {
    open3d::visualization::DrawGeometries({ //                std::make_shared<open3d::geometry::PointCloud>(hybrid_pcd),
                                            std::make_shared<open3d::geometry::TriangleMesh>(hybrid_mesh) });
  }
}

int main2(int argc, char** argv)
{
  open3d::geometry::PointCloud model_pcd, scan_pcd, output_pcd1, output_pcd2, output_pcd3;
  open3d::geometry::TriangleMesh output_mesh1, output_mesh2, output_mesh3;
  std::vector<open3d::geometry::PointCloud> seams;

  // argv[1] is model pcd
  open3d::io::ReadPointCloud(argv[1], model_pcd);
  model_pcd.PaintUniformColor(Eigen::Vector3d(0.5, 0.5, 1.0));

  // argv[2] is scan pcd
  open3d::io::ReadPointCloud(argv[2], scan_pcd);
  scan_pcd.PaintUniformColor(Eigen::Vector3d(1.0, 0.5, 0.5));

  // argv[3] to argv[-3] are seams
  for (int si = 3; si < argc - 3; si++)
  {
    open3d::geometry::PointCloud _seam;
    open3d::io::ReadPointCloud(argv[si], _seam);
    seams.push_back(_seam);
  }

  // argv[-3] is double for scoop_radius
  double scoop_radius = std::atof(argv[argc - 3]);

  // argv[-2] is bool for keep_full_model
  bool keep_full_model = std::string(argv[argc - 2]) == "keep_full_model";

  // argv[-1] is bool for visualization
  bool viz = std::string(argv[argc - 1]) == "viz";

  std::cout << "\n\nmakeHybridPointCloud" << std::endl;
  open3d::geometry::TriangleMesh hybrid_mesh;
  makeHybridPointCloud(scan_pcd, model_pcd, seams, output_pcd1, hybrid_mesh, false, scoop_radius, 0.8, viz);
  //    meshIt(output_pcd1, output_mesh1, 7.0);
  std::cout << std::endl;
  open3d::io::WritePointCloud("./hybrid_pcd.ply", output_pcd1);
  open3d::io::WriteTriangleMesh("./hybrid_mesh.ply", hybrid_mesh);

}

void scan_density_heatmap()
{
  std::string composite_normal_path = "/home/pmitra/data/valmont/86_1129_2_1644514426588/cloud_composite_normals.ply";
//  std::string composite_normal_path = "/home/pmitra/data/valmont/84_1111_2_1644429204689/cloud_composite_normals.ply";

// poisson reconstruction params
  double density_threshold = 0.0;
  size_t depth = 10;
  double weight = 5;
  size_t width = 0;
  double scale = 1.1;
  bool linear_fit = false;



  open3d::geometry::PointCloud pcd_hybrid_cloud;
  open3d::geometry::TriangleMesh mesh_hybrid_cloud;

  open3d::io::ReadPointCloud(composite_normal_path, pcd_hybrid_cloud);

  std::cout << "Meshing..." << std::endl;
  path_poisson::TriangleMesh meshed_output;
  path_poisson::MeshOptions mesh_ops;
  mesh_ops.verbose = false;
  path_poisson::PointCloud input_poisson_cloud;
//
//
  input_poisson_cloud.points_ = pcd_hybrid_cloud.points_;
  input_poisson_cloud.normals_ = pcd_hybrid_cloud.normals_;
//  std::cout << input_poisson_cloud.points_.size() << "  ---  " << input_poisson_cloud.normals_.size() << "\n";
  auto output_poisson_reconstruction = path_poisson::createPoissonMeshFromPointCloud(input_poisson_cloud, depth, weight, width, scale, linear_fit, mesh_ops);
//  meshIt(pcd_hybrid_cloud, mesh_hybrid_cloud);
  meshed_output = std::get<0>(output_poisson_reconstruction);
  mesh_hybrid_cloud.adjacency_list_ = meshed_output.adjacency_list_;
  mesh_hybrid_cloud.vertices_ = meshed_output.vertices_;
  mesh_hybrid_cloud.vertex_normals_ = meshed_output.vertex_normals_;
  mesh_hybrid_cloud.triangles_ = meshed_output.triangles_;
  mesh_hybrid_cloud.triangle_normals_ = meshed_output.triangle_normals_;
//
  std::cout<<mesh_hybrid_cloud.vertices_.size()<<std::endl;
//
  mesh_hybrid_cloud = mesh_hybrid_cloud.ComputeTriangleNormals(false);

//  open3d::visualization::DrawGeometries({ std::make_shared<open3d::geometry::TriangleMesh>(mesh_hybrid_cloud) });
//
//
  auto densities = std::get<1>(output_poisson_reconstruction);
  std::cout<<densities.size()<<std::endl;
//
  //create density mesh
  open3d::geometry::TriangleMesh density_mesh;
  density_mesh.adjacency_list_ = mesh_hybrid_cloud.adjacency_list_;
  density_mesh.vertices_ = mesh_hybrid_cloud.vertices_;
  density_mesh.vertex_normals_ = mesh_hybrid_cloud.vertex_normals_;
  density_mesh.triangles_ = mesh_hybrid_cloud.triangles_;
  density_mesh.triangle_normals_ = mesh_hybrid_cloud.triangle_normals_;

  float min_density = *min_element(densities.begin(), densities.end());
  float max_density = *max_element(densities.begin(), densities.end());

  std::vector<Eigen::Vector3d> density_based_colors(density_mesh.vertices_.size());
  for(auto iDensity = 0; iDensity<densities.size(); iDensity++)
  {
     density_based_colors[iDensity][0] = ((densities[iDensity] - min_density)/(max_density - min_density));
     density_based_colors[iDensity][1] = ((densities[iDensity] - min_density)/(max_density - min_density));
     density_based_colors[iDensity][2] = 0.9;
     //std::cout<<density_based_colors[iDensity][0]<<std::endl;
  }
  density_mesh.vertex_colors_ = density_based_colors;
//  open3d::visualization::DrawGeometries({ std::make_shared<open3d::geometry::TriangleMesh>(density_mesh) });
  open3d::io::WriteTriangleMesh("/home/pmitra/density_mesh.ply", density_mesh);


  //Building KDTree of mesh vertices to extract densities of triangles near the seam
  /*
   * Steps:
   * Keep populating the points in a vector
   * Remove duplicate points
   */

  //load seam point_cloud
  std::string seam_point_cloud_path = "/home/pmitra/data/valmont/86_1129_2_1644514426588/tf_feature_normals_0.ply";
  open3d::geometry::PointCloud seam_point_cloud;
  open3d::io::ReadPointCloud(seam_point_cloud_path, seam_point_cloud);

  auto mesh_hybrid_cloud_scan_kdtree = std::make_shared<open3d::geometry::KDTreeFlann>();
  std::cout<<"Creating KDTree for hybrid mesh...\n";
  mesh_hybrid_cloud_scan_kdtree->SetGeometry(mesh_hybrid_cloud);
  std::cout<<"Done\n";
  std::cout<<"what is happening";

  std::cout<<seam_point_cloud.points_.size();


  // evaluate along the seam
  std::vector<int>all_mesh_vertex_indices_nearest_to_seam;
  for (auto point : seam_point_cloud.points_)
    {
      // count number of points in the scan
      std::vector<int> mesh_vertex_indices_nearest_to_point;
      std::vector<double> mesh_vertex_distances_nearest_to_point;
      mesh_hybrid_cloud_scan_kdtree->SearchRadius(point, 0.02, mesh_vertex_indices_nearest_to_point, mesh_vertex_distances_nearest_to_point);
//      std::cout<<mesh_hybrid_cloud.vertices_[mesh_vertex_indices_nearest_to_point[0]]<<std::endl;
      for(int i = 0; i<mesh_vertex_indices_nearest_to_point.size(); i++)
      {
          all_mesh_vertex_indices_nearest_to_seam.push_back(mesh_vertex_indices_nearest_to_point[i]);
      }

    }
  std::cout<<all_mesh_vertex_indices_nearest_to_seam.size();


  /*
   * Steps:
   * 1) Create a TriangleMesh and initialize only with the vertex, vertex_normals, triangles, triangle_normals near the seam
   * 2) Fetch densities of these points
   * 3) Create density based colors only of these mesh vertices
   * 4) Bin these densities
   * 5) Repeat steps 1-4 for a good and a bad seam
   */

  open3d::geometry::TriangleMesh density_mesh_seam;
  density_mesh.adjacency_list_ = mesh_hybrid_cloud.adjacency_list_;
  density_mesh.vertices_ = mesh_hybrid_cloud.vertices_;
  density_mesh.vertex_normals_ = mesh_hybrid_cloud.vertex_normals_;
  density_mesh.triangles_ = mesh_hybrid_cloud.triangles_;
  density_mesh.triangle_normals_ = mesh_hybrid_cloud.triangle_normals_;





















}

int main(int argc, char** argv)
{
//    open3d::geometry::PointCloud scan_cloud;
//    open3d::io::ReadPointCloud("/home/pmitra/100_1282_2_1645371185020/cloud_composite_normals.ply", scan_cloud);
//
//    open3d::visualization::DrawGeometries({std::make_shared<open3d::geometry::PointCloud>(scan_cloud)});

//    open3d::geometry::PointCloud scan_cloud;
//    open3d::geometry::TriangleMesh triangle_mesh;
//    open3d::io::ReadPointCloud("/home/pmitra/100_1282_2_1645371185020/cloud_composite_normals.ply", scan_cloud);
//    meshIt(scan_cloud, triangle_mesh);
//    std::cout<<"meshed it"<<std::endl;
//    open3d::visualization::DrawGeometries({std::make_shared<open3d::geometry::TriangleMesh>(triangle_mesh)});

      scan_density_heatmap();


}