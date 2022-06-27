#include <poisson-recon/Reconstruction.h>

#include <Open3D/Open3D.h>
#include <Open3D/Geometry/PointCloud.h>

#include <vector>
#include <memory>
#include <iostream>
#include <fstream>

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



void write_density_to_file(std::vector<double> densities, std::string save_path)
{
    std::ofstream myfile (save_path);

    if (myfile.is_open())
    {
        for(int i = 0; i< densities.size(); i++)
        {
            myfile<<densities[i];
            myfile<<",";
        }
        myfile.close();
        std::cout<<"Written to text file\n";
    }
    else
    {
        std::cout << "Unable to open file";
    }
}

void scan_density_heatmap_exp()
{
// std::string composite_normal_path = "/home/pmitra/data/valmont/86_1129_2_1644514426588/cloud_composite_normals.ply";
//  std::string composite_normal_path = "/home/pmitra/data/western/253_1676_2_1648845471114/cloud_composite_normals.ply";
//    std::string data_root_path =  "/home/pmitra/workspace/density_data/bad_scans/";
//    std::string folder_path = "32_164_2_1651102426747";
//
//    std::string results_root_path = "/home/pmitra/workspace/density_results/bad/";
//
//    std::string composite_normal_path = data_root_path + folder_path + "/cloud_composite_normals.ply";
//    std::string seam_point_cloud_path = data_root_path +  folder_path + "/tf_feature_normals_0.ply";
//    std::string density_mesh_seam_save_path =  results_root_path + "density_mesh_seam_" + folder_path + ".ply";
//    std::string densities_textfile_save_path = results_root_path + "densities_seam_" + folder_path + ".txt";


std::string model_cloud_path = "/home/pmitra/data/valmont/79_1135_2_1644530181795/tf_model_cloud_.ply";
std::string seam_point_cloud_path =  "/home/pmitra/data/valmont/79_1135_2_1644530181795/tf_feature_normals_0.ply";
    std::string density_mesh_seam_save_path = "/home/pmitra/density_mesh_model_cloud.ply";
    std::string densities_textfile_save_path = "/home/pmitra/density_seam_model_cloud.txt";
// poisson reconstruction params
    double density_threshold = 0.0;
    size_t depth = 10;
    double weight = 5;
    size_t width = 0;
    double scale = 1.1;
    bool linear_fit = false;



    open3d::geometry::PointCloud pcd_hybrid_cloud;
    open3d::geometry::TriangleMesh mesh_hybrid_cloud;

    open3d::io::ReadPointCloud(model_cloud_path, pcd_hybrid_cloud);
    std::cout<<pcd_hybrid_cloud.points_.size()<<std::endl;
    auto tf_model_cloud = *pcd_hybrid_cloud.VoxelDownSample(0.0005);
    std::cout<<tf_model_cloud.points_.size()<<std::endl;



    std::cout << "Meshing..." << std::endl;
    path_poisson::TriangleMesh meshed_output;
    path_poisson::MeshOptions mesh_ops;
    mesh_ops.verbose = false;
    path_poisson::PointCloud input_poisson_cloud;
//
//
    input_poisson_cloud.points_ = tf_model_cloud.points_;
    input_poisson_cloud.normals_ = tf_model_cloud.normals_;
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
//  std::cout<<densities.size()<<std::endl;
////
//  //create density mesh
//  open3d::geometry::TriangleMesh density_mesh;
//  density_mesh.adjacency_list_ = mesh_hybrid_cloud.adjacency_list_;
//  density_mesh.vertices_ = mesh_hybrid_cloud.vertices_;
//  density_mesh.vertex_normals_ = mesh_hybrid_cloud.vertex_normals_;
//  density_mesh.triangles_ = mesh_hybrid_cloud.triangles_;
//  density_mesh.triangle_normals_ = mesh_hybrid_cloud.triangle_normals_;
//
//  float min_density = *min_element(densities.begin(), densities.end());
//  float max_density = *max_element(densities.begin(), densities.end());
//
//  std::vector<Eigen::Vector3d> density_based_colors(density_mesh.vertices_.size());
//  for(auto iDensity = 0; iDensity<densities.size(); iDensity++)
//  {
//     density_based_colors[iDensity][0] = ((densities[iDensity] - min_density)/(max_density - min_density));
//     density_based_colors[iDensity][1] = ((densities[iDensity] - min_density)/(max_density - min_density));
//     density_based_colors[iDensity][2] = 0.9;
//     //std::cout<<density_based_colors[iDensity][0]<<std::endl;
//  }
//  density_mesh.vertex_colors_ = density_based_colors;
////  open3d::visualization::DrawGeometries({ std::make_shared<open3d::geometry::TriangleMesh>(density_mesh) });
//  open3d::io::WriteTriangleMesh("/home/pmitra/density_mesh.ply", density_mesh);


    //Building KDTree of mesh vertices to extract densities of triangles near the seam
    /*
     * Steps:
     * Keep populating the points in a vector
     * Remove duplicate points
     */

    //load seam point_cloud
    open3d::geometry::PointCloud seam_point_cloud;
    open3d::io::ReadPointCloud(seam_point_cloud_path, seam_point_cloud);

    auto mesh_hybrid_cloud_scan_kdtree = std::make_shared<open3d::geometry::KDTreeFlann>();
    std::cout<<"Creating KDTree for hybrid mesh...\n";
    mesh_hybrid_cloud_scan_kdtree->SetGeometry(mesh_hybrid_cloud);
    std::cout<<"Done\n";

    std::cout<<seam_point_cloud.points_.size()<<std::endl;


    // evaluate along the seam
    std::vector<int>all_mesh_vertex_indices_nearest_to_seam;
    for (auto point : seam_point_cloud.points_)
    {
        // count number of points in the scan
        std::vector<int> mesh_vertex_indices_nearest_to_point;
        std::vector<double> mesh_vertex_distances_nearest_to_point;
        mesh_hybrid_cloud_scan_kdtree->SearchRadius(point, 0.02, mesh_vertex_indices_nearest_to_point, mesh_vertex_distances_nearest_to_point);
        for(int i = 0; i<mesh_vertex_indices_nearest_to_point.size(); i++)
        {
            all_mesh_vertex_indices_nearest_to_seam.push_back(mesh_vertex_indices_nearest_to_point[i]);
        }

    }

    std::sort(all_mesh_vertex_indices_nearest_to_seam.begin(), all_mesh_vertex_indices_nearest_to_seam.end());
    all_mesh_vertex_indices_nearest_to_seam.erase( unique( all_mesh_vertex_indices_nearest_to_seam.begin(), all_mesh_vertex_indices_nearest_to_seam.end() ), all_mesh_vertex_indices_nearest_to_seam.end() );

    for(int i = 0; i< all_mesh_vertex_indices_nearest_to_seam.size() - 1; i++)
    {
        if (all_mesh_vertex_indices_nearest_to_seam[i] == all_mesh_vertex_indices_nearest_to_seam[i+1])
        {
            std::cout<<all_mesh_vertex_indices_nearest_to_seam[i]<<std::endl;

        }
    }
    std::cout<<all_mesh_vertex_indices_nearest_to_seam.size()<<std::endl;


//
//  std::cout<<all_mesh_vertex_indices_nearest_to_seam.size()<<std::endl;


    /*
     * Steps:
     * 1) Create a TriangleMesh and initialize only with the vertex, vertex_normals, triangles, triangle_normals near the seam
     * 2) Fetch densities of these points
     * 3) Create density based colors only of these mesh vertices
     * 4) Bin these densities
     * 5) Repeat steps 1-4 for a good and a bad seam
     */

//  open3d::geometry::TriangleMesh density_mesh_seam;
//  for(int i = 0; i<all_mesh_vertex_indices_nearest_to_seam.size(); i++)
//    {
//
//        density_mesh_seam.vertices_.push_back(mesh_hybrid_cloud.vertices_[all_mesh_vertex_indices_nearest_to_seam[i]]);
//        density_mesh_seam.vertex_normals_.push_back(mesh_hybrid_cloud.vertex_normals_[all_mesh_vertex_indices_nearest_to_seam[i]]);
//        density_mesh_seam.triangles_.push_back(mesh_hybrid_cloud.triangles_[all_mesh_vertex_indices_nearest_to_seam[i]]);
//        density_mesh_seam.triangle_normals_.push_back(mesh_hybrid_cloud.triangle_normals_[all_mesh_vertex_indices_nearest_to_seam[i]]);
//
//    }
//  std::cout<<"pushed density vertices";
//
//  std::vector<double>densities_seam;
//  for(int i = 0; i<all_mesh_vertex_indices_nearest_to_seam.size(); i++)
//  {
//      densities_seam.push_back(densities[all_mesh_vertex_indices_nearest_to_seam[i]]);
//  }
//
//    float min_density_seam = *min_element(densities_seam.begin(), densities_seam.end());
//    float max_density_seam = *max_element(densities_seam.begin(), densities_seam.end());
//
//    std::vector<Eigen::Vector3d> density_based_colors_seam(density_mesh_seam.vertices_.size());
//    for(auto iDensity = 0; iDensity<densities_seam.size(); iDensity++)
//    {
//        density_based_colors_seam[iDensity][0] = ((densities_seam[iDensity] - min_density_seam)/(max_density_seam - min_density_seam));
//        density_based_colors_seam[iDensity][1] = ((densities_seam[iDensity] - min_density_seam)/(max_density_seam - min_density_seam));
//        density_based_colors_seam[iDensity][2] = 0.9;
//        //std::cout<<density_based_colors[iDensity][0]<<std::endl;
//    }
//    density_mesh_seam.vertex_colors_ = density_based_colors_seam;
////  open3d::visualization::DrawGeometries({ std::make_shared<open3d::geometry::TriangleMesh>(density_mesh) });
//    open3d::io::WriteTriangleMesh("/home/pmitra/density_mesh_seam.ply", density_mesh_seam);

    open3d::geometry::TriangleMesh density_mesh_seam(mesh_hybrid_cloud);
    std::vector<bool> non_seam_mask(mesh_hybrid_cloud.vertices_.size(), true);
    for (size_t i = 0; i < all_mesh_vertex_indices_nearest_to_seam.size(); i++)
    {
        non_seam_mask[all_mesh_vertex_indices_nearest_to_seam[i]] = false ;
    }

    density_mesh_seam.RemoveVerticesByMask(non_seam_mask);
    std::cout<<density_mesh_seam.vertices_.size()<<std::endl;

//    std::cout<<density_mesh_seam.HasVertexColors()<< "CP 0 -------"<<std::endl;
    std::cout<<"vertices removed"<<std::endl;
    std::vector<double>densities_seam;
    for(size_t i = 0; i<all_mesh_vertex_indices_nearest_to_seam.size(); i++)
    {
        densities_seam.push_back(densities[all_mesh_vertex_indices_nearest_to_seam[i]]);
    }
    std::cout<<"seam densitites populated  "<<densities_seam.size()<<std::endl;
//    std::cout<<density_mesh_seam.HasVertexColors()<< "CP 1 -------"<<std::endl;
    float min_density_seam = *min_element(densities_seam.begin(), densities_seam.end());
    float max_density_seam = *max_element(densities_seam.begin(), densities_seam.end());

    std::vector<Eigen::Vector3d> density_based_colors_seam(density_mesh_seam.vertices_.size());

    std::cout<<densities_seam.size()<<std::endl;
    std::cout<<density_mesh_seam.vertices_.size()<<std::endl;
    std::cout<<density_based_colors_seam.size()<<std::endl;
    for(size_t iDensity = 0; iDensity<density_based_colors_seam.size(); iDensity++)
    {
        density_based_colors_seam[iDensity][0] = ((densities_seam[iDensity] - min_density_seam)/(max_density_seam - min_density_seam));
        density_based_colors_seam[iDensity][1] = ((densities_seam[iDensity] - min_density_seam)/(max_density_seam - min_density_seam));
        density_based_colors_seam[iDensity][2] = 0.9;
    }

    density_mesh_seam.vertex_colors_ = density_based_colors_seam;

    open3d::visualization::DrawGeometries({ std::make_shared<open3d::geometry::TriangleMesh>(density_mesh_seam) });
    open3d::io::WriteTriangleMesh(density_mesh_seam_save_path, density_mesh_seam);

    write_density_to_file(densities_seam, densities_textfile_save_path);

}

void scan_density_heatmap(std::string data_folder_path)
{
 std::string composite_normal_path = "/home/pmitra/workspace/density_data/good_scans/41_203_2_1651583400424/cloud_composite_normals.ply";
 std::string seam_point_cloud_path =  "/home/pmitra/workspace/density_data/good_scans/41_203_2_1651583400424/tf_feature_normals_0.ply";

// std::string composite_normal_path = "/home/pmitra/data/western/253_1676_2_1648845471114/cloud_composite_normals.ply";
//std::string data_root_path =  "/home/pmitra/workspace/density_data/bad_scans/";

//std::string  data_root_path =  "/home/pmitra/data/valmont/bulk_data/";

//std::string results_root_path = "/home/pmitra/workspace/density_results/bulk_data/";
//std::string composite_normal_path = data_root_path + data_folder_path + "/cloud_composite_normals.ply";
//std::string seam_point_cloud_path = data_root_path +  data_folder_path + "/tf_feature_normals_0.ply";
//std::string density_mesh_seam_save_path =  results_root_path + "density_mesh_seam_" + data_folder_path + ".ply";
//std::string densities_textfile_save_path = results_root_path + "densities_seam_" + data_folder_path + ".txt";
std::string density_mesh_seam_save_path =  "/home/pmitra/density_mesh_seam_good.ply";
std::string densities_textfile_save_path = "/home/pmitra/density_mesh_seam_good.txt";

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
  //Building KDTree of mesh vertices to extract densities of triangles near the seam
  /*
   * Steps:
   * Keep populating the points in a vector
   * Remove duplicate points
   */

  //load seam point_cloud
  open3d::geometry::PointCloud seam_point_cloud;
  open3d::io::ReadPointCloud(seam_point_cloud_path, seam_point_cloud);

  auto mesh_hybrid_cloud_scan_kdtree = std::make_shared<open3d::geometry::KDTreeFlann>();
  std::cout<<"Creating KDTree for hybrid mesh...\n";
  mesh_hybrid_cloud_scan_kdtree->SetGeometry(mesh_hybrid_cloud);
  std::cout<<"Done\n";

  std::cout<<seam_point_cloud.points_.size()<<std::endl;


  // evaluate along the seam
  std::vector<int>all_mesh_vertex_indices_nearest_to_seam;
  for (auto point : seam_point_cloud.points_)
    {

      // count number of points in the scan
      std::vector<int> mesh_vertex_indices_nearest_to_point;
      std::vector<double> mesh_vertex_distances_nearest_to_point;
      mesh_hybrid_cloud_scan_kdtree->SearchRadius(point, 0.02, mesh_vertex_indices_nearest_to_point, mesh_vertex_distances_nearest_to_point);
      for(int i = 0; i<mesh_vertex_indices_nearest_to_point.size(); i++)
      {
          all_mesh_vertex_indices_nearest_to_seam.push_back(mesh_vertex_indices_nearest_to_point[i]);
      }

    }
    // removing duplicates
    // source : https://stackoverflow.com/questions/1041620/whats-the-most-efficient-way-to-erase-duplicates-and-sort-a-vector

    std::sort(all_mesh_vertex_indices_nearest_to_seam.begin(), all_mesh_vertex_indices_nearest_to_seam.end());
    all_mesh_vertex_indices_nearest_to_seam.erase( unique( all_mesh_vertex_indices_nearest_to_seam.begin(), all_mesh_vertex_indices_nearest_to_seam.end() ), all_mesh_vertex_indices_nearest_to_seam.end() );

    // checking if any duplicates are present
    for(int i = 0; i< all_mesh_vertex_indices_nearest_to_seam.size() - 1; i++)
      {
          if (all_mesh_vertex_indices_nearest_to_seam[i] == all_mesh_vertex_indices_nearest_to_seam[i+1])
          {
              std::cout<<all_mesh_vertex_indices_nearest_to_seam[i]<<std::endl;

          }
      }
  std::cout<<all_mesh_vertex_indices_nearest_to_seam.size()<<std::endl;


//
//  std::cout<<all_mesh_vertex_indices_nearest_to_seam.size()<<std::endl;




//  open3d::geometry::TriangleMesh density_mesh_seam;
//  for(int i = 0; i<all_mesh_vertex_indices_nearest_to_seam.size(); i++)
//    {
//
//        density_mesh_seam.vertices_.push_back(mesh_hybrid_cloud.vertices_[all_mesh_vertex_indices_nearest_to_seam[i]]);
//        density_mesh_seam.vertex_normals_.push_back(mesh_hybrid_cloud.vertex_normals_[all_mesh_vertex_indices_nearest_to_seam[i]]);
//        density_mesh_seam.triangles_.push_back(mesh_hybrid_cloud.triangles_[all_mesh_vertex_indices_nearest_to_seam[i]]);
//        density_mesh_seam.triangle_normals_.push_back(mesh_hybrid_cloud.triangle_normals_[all_mesh_vertex_indices_nearest_to_seam[i]]);
//
//    }
//  std::cout<<"pushed density vertices";
//
//  std::vector<double>densities_seam;
//  for(int i = 0; i<all_mesh_vertex_indices_nearest_to_seam.size(); i++)
//  {
//      densities_seam.push_back(densities[all_mesh_vertex_indices_nearest_to_seam[i]]);
//  }
//
//    float min_density_seam = *min_element(densities_seam.begin(), densities_seam.end());
//    float max_density_seam = *max_element(densities_seam.begin(), densities_seam.end());
//
//    std::vector<Eigen::Vector3d> density_based_colors_seam(density_mesh_seam.vertices_.size());
//    for(auto iDensity = 0; iDensity<densities_seam.size(); iDensity++)
//    {
//        density_based_colors_seam[iDensity][0] = ((densities_seam[iDensity] - min_density_seam)/(max_density_seam - min_density_seam));
//        density_based_colors_seam[iDensity][1] = ((densities_seam[iDensity] - min_density_seam)/(max_density_seam - min_density_seam));
//        density_based_colors_seam[iDensity][2] = 0.9;
//        //std::cout<<density_based_colors[iDensity][0]<<std::endl;
//    }
//    density_mesh_seam.vertex_colors_ = density_based_colors_seam;
//  open3d::visualization::DrawGeometries({ std::make_shared<open3d::geometry::TriangleMesh>(density_mesh) });
//    open3d::io::WriteTriangleMesh("/home/pmitra/density_mesh_seam.ply", density_mesh_seam);


/*
   * Steps:
   * 1) Create a TriangleMesh
   * 2) Fetch densities of points only near the seam
   * 3) Create density based colors only of these mesh vertices
   * 4) Bin these densities
   * 5) Repeat steps 1-4 for a good and a bad seam
   */

    open3d::geometry::TriangleMesh density_mesh_seam(mesh_hybrid_cloud);
    std::vector<bool> non_seam_mask(mesh_hybrid_cloud.vertices_.size(), true);
    for (size_t i = 0; i < all_mesh_vertex_indices_nearest_to_seam.size(); i++)
    {
        non_seam_mask[all_mesh_vertex_indices_nearest_to_seam[i]] = false ;
    }

    density_mesh_seam.RemoveVerticesByMask(non_seam_mask);
    std::cout<<density_mesh_seam.vertices_.size()<<std::endl;

//    std::cout<<density_mesh_seam.HasVertexColors()<< "CP 0 -------"<<std::endl;
    std::cout<<"vertices removed"<<std::endl;
      std::vector<double>densities_seam;
  for(size_t i = 0; i<all_mesh_vertex_indices_nearest_to_seam.size(); i++)
  {
      densities_seam.push_back(densities[all_mesh_vertex_indices_nearest_to_seam[i]]);
  }
  std::cout<<"seam densitites populated  "<<densities_seam.size()<<std::endl;
//    std::cout<<density_mesh_seam.HasVertexColors()<< "CP 1 -------"<<std::endl;
    float min_density_seam = *min_element(densities_seam.begin(), densities_seam.end());
    float max_density_seam = *max_element(densities_seam.begin(), densities_seam.end());

    std::vector<Eigen::Vector3d> density_based_colors_seam(density_mesh_seam.vertices_.size());

    std::cout<<densities_seam.size()<<std::endl;
    std::cout<<density_mesh_seam.vertices_.size()<<std::endl;
    std::cout<<density_based_colors_seam.size()<<std::endl;
    for(size_t iDensity = 0; iDensity<density_based_colors_seam.size(); iDensity++)
    {
        density_based_colors_seam[iDensity][0] = ((densities_seam[iDensity] - min_density_seam)/(max_density_seam - min_density_seam));
        density_based_colors_seam[iDensity][1] = ((densities_seam[iDensity] - min_density_seam)/(max_density_seam - min_density_seam));
        density_based_colors_seam[iDensity][2] = 0.9;
    }

    density_mesh_seam.vertex_colors_ = density_based_colors_seam;

//  open3d::visualization::DrawGeometries({ std::make_shared<open3d::geometry::TriangleMesh>(density_mesh_seam) });
  open3d::io::WriteTriangleMesh(density_mesh_seam_save_path, density_mesh_seam);

  write_density_to_file(densities_seam, densities_textfile_save_path);

}

void scan_density_heatmap_loop(std::vector<std::string> dataset_names)
{
/*
 Steps:
 1) List all dataset names in a folder
 2) Push to a vector of strings
 3) Fetch each element of the vector
 4) Pass to scan_density_heatmap()

 */
    std::cout<<"Processing datasets..:"<<dataset_names.size()<<std::endl;

  for(int i = 0; i<dataset_names.size(); i++)
  {
      std::cout<<dataset_names[i]<<std::endl;
      scan_density_heatmap(dataset_names[i]);
  }

}

void create_low_density_point_clouds(std::string cloud_composite_normal_path, std::string seam_point_cloud_path)
{
    /**
     * Steps:
     * Load a good mesh (one with high density of points)
     * Search around the seam with a fixed radius to collect points around the seam
     * Do a RemoveVerticesbyMask() of the above points from the original point cloud - call this point_cloud_with_hole
     * Collect these points and initialize it to an empty point cloud - call this cropped_point_cloud
     * Do a voxel downsampling of croppped_point_cloud
     * low_density_point_cloud = point_cloud_with_hole + cropped_point_cloud
     */

    //Load a good mesh
    open3d::geometry::PointCloud pcd_good_scan;
    open3d::io::ReadPointCloud(cloud_composite_normal_path, pcd_good_scan);
    std::cout<<pcd_good_scan.points_.size()<<"\n";

//    open3d::geometry::TriangleMesh mesh_good_scan;

    // Search around the seam with a fixed radius to collect points around the seam
    // load seam point_cloud
    open3d::geometry::PointCloud seam_point_cloud;
    open3d::io::ReadPointCloud(seam_point_cloud_path, seam_point_cloud);

    auto pcd_good_scan_kdtree = std::make_shared<open3d::geometry::KDTreeFlann>();
    std::cout<<"Creating KDTree for hybrid mesh...\n";
    pcd_good_scan_kdtree->SetGeometry(pcd_good_scan);
    std::cout<<"Done\n";


    // evaluate along the seam
    std::vector<size_t>all_pcd_vertex_indices_nearest_to_seam;
    for (auto point : seam_point_cloud.points_)
    {

        // count number of points in the scan
        std::vector<int> pcd_vertex_indices_nearest_to_point;
        std::vector<double> pcd_vertex_distances_nearest_to_point;
        pcd_good_scan_kdtree->SearchRadius(point, 0.02, pcd_vertex_indices_nearest_to_point, pcd_vertex_distances_nearest_to_point);
        for(size_t i = 0; i<pcd_vertex_indices_nearest_to_point.size(); i++)
        {
            all_pcd_vertex_indices_nearest_to_seam.push_back(pcd_vertex_indices_nearest_to_point[i]);
        }

    }
    // removing duplicates
    // source : https://stackoverflow.com/questions/1041620/whats-the-most-efficient-way-to-erase-duplicates-and-sort-a-vector

    std::sort(all_pcd_vertex_indices_nearest_to_seam.begin(), all_pcd_vertex_indices_nearest_to_seam.end());
    all_pcd_vertex_indices_nearest_to_seam.erase( unique( all_pcd_vertex_indices_nearest_to_seam.begin(), all_pcd_vertex_indices_nearest_to_seam.end() ), all_pcd_vertex_indices_nearest_to_seam.end() );

    // checking if any duplicates are present
    for(size_t i = 0; i< all_pcd_vertex_indices_nearest_to_seam.size() - 1; i++)
    {
        if (all_pcd_vertex_indices_nearest_to_seam[i] == all_pcd_vertex_indices_nearest_to_seam[i+1])
        {
            std::cout<<all_pcd_vertex_indices_nearest_to_seam[i]<<std::endl;

        }
    }
    std::cout<<all_pcd_vertex_indices_nearest_to_seam.size()<<std::endl;

//    open3d::geometry::PointCloud cropped_point_cloud;
//    auto cropped_point_cloud_pointer = std::make_shared<open3d::geometry::PointCloud>(cropped_point_cloud);
//    auto pcd_good_scan_pointer = std::make_shared<open3d::geometry::PointCloud>(pcd_good_scan);

    auto cropped_seam_point_cloud_pointer = pcd_good_scan.SelectDownSample(all_pcd_vertex_indices_nearest_to_seam);
    open3d::io::WritePointCloud("/home/pmitra/just_seam_points.ply", *cropped_seam_point_cloud_pointer);
    std::cout<<cropped_seam_point_cloud_pointer->points_.size()<<"\n";


    auto downsampled_cropped_seam_point_cloud_pointer = cropped_seam_point_cloud_pointer->VoxelDownSample(0.003);
    open3d::io::WritePointCloud("/home/pmitra/downsampled.ply", *downsampled_cropped_seam_point_cloud_pointer);

    auto point_cloud_without_seam = pcd_good_scan.SelectDownSample(all_pcd_vertex_indices_nearest_to_seam, true);
    open3d::io::WritePointCloud("/home/pmitra/without_seam_points.ply", *point_cloud_without_seam);
    std::cout<<point_cloud_without_seam->points_.size()<<"\n";

    auto low_density_point_cloud = *downsampled_cropped_seam_point_cloud_pointer + *point_cloud_without_seam;
    open3d::io::WritePointCloud("/home/pmitra/low_density_point_cloud.ply", low_density_point_cloud);

}
int main(int argc, char** argv)
{

//      scan_density_heatmap();

//   std::vector<std::string> dataset_names = {"32_164_2_1651102426747", "14_125_2_1650751255276", "17_138_2_1650762128484", "119_440_2_1654200602795", "41_200_2_1651519784634", "100_1293_2_1645472303462", "126_1537_2_1647448143837", "21_153_2_1650921250169", "133_484_2_1654783343712", "100_424_2_1654093694420", "106_427_2_1654104316596", "126_1536_2_1647447300179", "37_177_2_1651250036542", "103_1286_2_1645386899290", "111_434_2_1654183494281", "133_485_2_1654790152363", "35_173_2_1651177962783", "123_1385_2_1645767066044", "119_1332_2_1645665542075", "125_448_2_1654288755546", "132_465_2_1654610403045", "133_473_2_1654694341514", "121_1373_2_1645753841371", "34_170_2_1651169342585", "152_537_2_1655327662782", "120_442_2_1654209228970", "114_435_2_1654188453105", "109_1295_2_1645484936228", "133_479_2_1654719652773", "132_467_2_1654616718525", "30_157_2_1651014006148", "27_155_2_1650939358146", "33_412_2_1637697176316", "115_1323_2_1645653916113", "115_1336_2_1645670525515", "34_169_2_1651165533495", "133_482_2_1654738480824",  "133_475_2_1654704359620", "14_141_2_1650814667642", "102_422_2_1654083712815", "34_171_2_1651172021259", "115_1316_2_1645601671594", "33_219_2_1651844796076", "35_194_2_1651505090715", "3_44_2_1635464027141", "124_446_2_1654280872834", "123_1384_2_1645766060903", "130_459_2_1654546984467", "35_192_2_1651498177882", "100_1282_2_1645371185020", "3_38_2_1635462997523", "115_1322_2_1645638040851", "119_1333_2_1645666089307", "136_1676_2_1648069922332", "119_1328_2_1645663502625", "121_1388_2_1645775029292", "122_1376_2_1645755104617", "41_204_2_1651589492885", "104_1287_2_1645395063800", "132_469_2_1654622462426", "120_1369_2_1645742514344", "155_545_2_1655424897923", "3_51_2_1635466299317", "119_1334_2_1645666714530", "111_1303_2_1645555872951", "132_472_2_1654633805709", "130_1662_2_1647892198312", "132_468_2_1654619149021", "121_1372_2_1645753222470", "41_202_2_1651526755533", "131_457_2_1654538098335",  "126_1542_2_1647454415088", "132_471_2_1654630673613", "133_486_2_1654795073021", "133_474_2_1654699607633", "39_459_2_1638809412930", "104_425_2_1654097071410", "38_185_2_1651273730923", "41_203_2_1651583400424", "39_458_2_1638804660218", "120_441_2_1654204924028", "41_205_2_1651594879216", "126_1656_2_1647810032012", "110_1301_2_1645542662381", "121_1387_2_1645773344818", "133_483_2_1654779982666", "25_150_2_1650855911220", "130_458_2_1654541821716", "105_426_2_1654100680366", "19_432_2_1638403658063", "39_195_2_1651511795176", "154_542_2_1655388895452", "21_148_2_1650850844253", "3_30_2_1635454531169", "109_431_2_1654123875884", "18_142_2_1650824895103", "133_478_2_1654716349277", "11_115_2_1650730475437", "109_443_2_1654216331780", "120_1339_2_1645712022564", "133_476_2_1654708001397", "130_1661_2_1647878994333", "133_480_2_1654726058154", "115_1313_2_1645592453586", "119_1330_2_1645664427375", "122_444_2_1654265779776", "123_1383_2_1645765648153", "103_445_2_1654270703712", "132_470_2_1654627083500", "33_167_2_1651160257906", "126_452_2_1654519100144", "129_455_2_1654529702489", "155_544_2_1655408158599", "111_1308_2_1645577604721", "19_428_2_1638387805631", "121_1381_2_1645765176140", "103_423_2_1654088407479", "126_1532_2_1647444555310", "14_139_2_1650812607537", "121_1390_2_1645803905460", "33_168_2_1651162896446", "121_1380_2_1645764845467", "119_1329_2_1645663912062", "128_454_2_1654526235268", "3_36_2_1635462684929", "126_1533_2_1647445144116", "11_210_2_1636066321276", "119_1331_2_1645664985280", "32_165_2_1651154800523", "119_1391_2_1645820632802", "152_539_2_1655382347311",  "30_163_2_1651095610058", "19_629_2_1639577102772", "3_46_2_1635464620533", "39_196_2_1651516093488", "14_126_2_1650751958525", "130_456_2_1654533818773", "30_166_2_1651157409469", "3_381_2_1637334834034", "121_1379_2_1645764243255", "40_187_2_1651331750827", "35_193_2_1651502753799"};
//   scan_density_heatmap_loop(dataset_names);

    std::string composite_normal_path = "/home/pmitra/workspace/density_data/good_scans/41_203_2_1651583400424/cloud_composite_normals.ply";
    std::string seam_point_cloud_path =  "/home/pmitra/workspace/density_data/good_scans/41_203_2_1651583400424/tf_feature_normals_0.ply";

//    create_low_density_point_clouds(composite_normal_path, seam_point_cloud_path);
    scan_density_heatmap("garbage_value/");

}