#include "stdafx.h"
#include <pcl/pcl_base.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <list>
#include <queue>
#include <math.h>
#include <time.h>

  /** \brief
    * Implements the well known Region Growing algorithm used for segmentation.
    * Description can be found in the article
    * "Segmentation of point clouds using smoothness constraint"
    * by T. Rabbania, F. A. van den Heuvelb, G. Vosselmanc.
    * In addition to residual test, the possibility to test curvature is added.
    */

inline bool
    comparePair (const std::pair<float, int> &i, const std::pair<float, int> &j)
{
    return (i.first < j.first);
}

  template <typename PointT, typename NormalT>
  class PCL_EXPORTS RegionGrowing : public pcl::PCLBase<PointT>
  {
    public:

      typedef pcl::search::Search <PointT> KdTree;
      typedef typename KdTree::Ptr KdTreePtr;
      typedef pcl::PointCloud <NormalT> Normal;
      typedef typename Normal::Ptr NormalPtr;
      typedef pcl::PointCloud <PointT> PointCloud;

      using PCLBase <PointT>::input_;
      using PCLBase <PointT>::indices_;
      using PCLBase <PointT>::initCompute;
      using PCLBase <PointT>::deinitCompute;

    public:

      /** \brief Constructor that sets default values for member variables. */
        RegionGrowing () :
          min_pts_per_cluster_ (1),
              max_pts_per_cluster_ (std::numeric_limits<int>::max ()),
              smooth_mode_flag_ (true),
              curvature_flag_ (true),
              residual_flag_ (false),
              theta_threshold_ (30.0f / 180.0f * static_cast<float> (M_PI)),
              residual_threshold_ (0.05f),
              curvature_threshold_ (0.05f),
              neighbour_radius_ (1.5),
              search_ (),
              normals_ (),
              point_neighbours_ (0),
              point_labels_ (0),
              normal_flag_ (true),
              num_pts_in_segment_ (0),
              clusters_ (0),
              number_of_segments_ (0)
          {
          }

      /** \brief This destructor destroys the cloud, normals and search method used for
        * finding KNN. In other words it frees memory.
        */
      virtual
      ~RegionGrowing ()
      {
                if (search_ != 0)
          search_.reset ();
      if (normals_ != 0)
          normals_.reset ();

      point_neighbours_.clear ();
      point_labels_.clear ();
      num_pts_in_segment_.clear ();
      clusters_.clear ();
      }


    protected:

      /** \brief Stores the minimum number of points that a cluster needs to contain in order to be considered valid. */
      int min_pts_per_cluster_;

      /** \brief Stores the maximum number of points that a cluster needs to contain in order to be considered valid. */
      int max_pts_per_cluster_;

      /** \brief Flag that signalizes if the smoothness constraint will be used. */
      bool smooth_mode_flag_;

      /** \brief If set to true then curvature test will be done during segmentation. */
      bool curvature_flag_;

      /** \brief If set to true then residual test will be done during segmentation. */
      bool residual_flag_;

      /** \brief Thershold used for testing the smoothness between points. */
      float theta_threshold_;

      /** \brief Thershold used in residual test. */
      float residual_threshold_;

      /** \brief Thershold used in curvature test. */
      float curvature_threshold_;

      /** \brief Number of neighbours to find. */
      float neighbour_radius_;


      /** \brief Serch method that will be used for KNN. */
      KdTreePtr search_;

      /** \brief Contains normals of the points that will be segmented. */
      NormalPtr normals_;

      /** \brief Contains neighbours of each point. */
      std::vector<std::vector<int> > point_neighbours_;

      /** \brief Point labels that tells to which segment each point belongs. */
      std::vector<int> point_labels_;

      /** \brief If set to true then normal/smoothness test will be done during segmentation.
        * It is always set to true for the usual region growing algorithm. It is used for turning on/off the test
        * for smoothness in the child class RegionGrowingRGB.*/
      bool normal_flag_;

      /** \brief Tells how much points each segment contains. Used for reserving memory. */
      std::vector<int> num_pts_in_segment_;

      /** \brief After the segmentation it will contain the segments. */
      std::vector <pcl::PointIndices> clusters_;

      /** \brief Stores the number of segments. */
      int number_of_segments_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW


  /** \brief This function is used as a comparator for sorting. */
   public:


   int
      RegionGrowing::getMinClusterSize ()
  {
      return (min_pts_per_cluster_);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   void
      RegionGrowing::setMinClusterSize (int min_cluster_size)
  {
      min_pts_per_cluster_ = min_cluster_size;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   int
      RegionGrowing::getMaxClusterSize ()
  {
      return (max_pts_per_cluster_);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   void
      RegionGrowing::setMaxClusterSize (int max_cluster_size)
  {
      max_pts_per_cluster_ = max_cluster_size;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   bool
      RegionGrowing::getSmoothModeFlag () const
  {
      return (smooth_mode_flag_);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   void
      RegionGrowing::setSmoothModeFlag (bool value)
  {
      smooth_mode_flag_ = value;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   bool
      RegionGrowing::getCurvatureTestFlag () const
  {
      return (curvature_flag_);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   void
      RegionGrowing::setCurvatureTestFlag (bool value)
  {
      curvature_flag_ = value;

      if (curvature_flag_ == false && residual_flag_ == false)
          residual_flag_ = true;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   bool
      RegionGrowing::getResidualTestFlag () const
  {
      return (residual_flag_);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   void
      RegionGrowing::setResidualTestFlag (bool value)
  {
      residual_flag_ = value;

      if (curvature_flag_ == false && residual_flag_ == false)
          curvature_flag_ = true;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   float
      RegionGrowing::getSmoothnessThreshold () const
  {
      return (theta_threshold_);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   void
      RegionGrowing::setSmoothnessThreshold (float theta)
  {
      theta_threshold_ = theta;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   float
      RegionGrowing::getResidualThreshold () const
  {
      return (residual_threshold_);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   void
      RegionGrowing::setResidualThreshold (float residual)
  {
      residual_threshold_ = residual;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   float
      RegionGrowing::getCurvatureThreshold () const
  {
      return (curvature_threshold_);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   void
      RegionGrowing::setCurvatureThreshold (float curvature)
  {
      curvature_threshold_ = curvature;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   float
      RegionGrowing::getNumberOfNeighbours () const
  {
      return (neighbour_radius_);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   void
      RegionGrowing::setNumberOfNeighbours (float neighbour_radius)
  {
      neighbour_radius_ = neighbour_radius;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   typename RegionGrowing<PointT, NormalT>::KdTreePtr
      RegionGrowing::getSearchMethod () const
  {
      return (search_);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   void
      RegionGrowing::setSearchMethod (const KdTreePtr& tree)
  {
      if (search_ != 0)
          search_.reset ();

      search_ = tree;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   typename NormalPtr
      RegionGrowing::getInputNormals () const
  {
      return (normals_);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   void
      RegionGrowing::setInputNormals (const NormalPtr& norm)
  {
      if (normals_ != 0)
          normals_.reset ();

      normals_ = norm;
  }
   void
      RegionGrowing::initialize(CloudPtr cloud,NormalPtr normals)
   {
       float SmoothnessThreshold=15.0;
       float CurvatureThreshold=0.1;
       float ResidualThreshold=0.5;
       float radius=1.5;
       pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_method (new pcl::search::KdTree<pcl::PointXYZ> ());
       setMinClusterSize (50);
       setMaxClusterSize (1000000);
       setSearchMethod (tree_method);
       setNumberOfNeighbours (radius);
       setInputCloud (cloud);
       setInputNormals (normals);
       setSmoothnessThreshold (SmoothnessThreshold/ 180.0 * M_PI);
       setCurvatureThreshold (CurvatureThreshold);
       setResidualThreshold(ResidualThreshold);
       setResidualTestFlag(true);
   }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   void
      RegionGrowing::extract (std::vector <pcl::PointIndices>& clusters)
  {
      clusters_.clear ();
      clusters.clear ();
      point_neighbours_.clear ();
      point_labels_.clear ();
      num_pts_in_segment_.clear ();
      number_of_segments_ = 0;

      bool segmentation_is_possible = initCompute ();
      if ( !segmentation_is_possible )
      {
          deinitCompute ();
          return;
      }

      segmentation_is_possible = prepareForSegmentation ();
      if ( !segmentation_is_possible )
      {
          deinitCompute ();
          return;
      }

      findPointNeighbours ();
      applySmoothRegionGrowingAlgorithm ();
      assembleRegions ();

      clusters.resize (clusters_.size ());
      std::vector<pcl::PointIndices>::iterator cluster_iter_input = clusters.begin ();
      for (std::vector<pcl::PointIndices>::const_iterator cluster_iter = clusters_.begin (); cluster_iter != clusters_.end (); cluster_iter++)
      {
          if ((static_cast<int> (cluster_iter->indices.size ()) >= min_pts_per_cluster_) &&
              (static_cast<int> (cluster_iter->indices.size ()) <= max_pts_per_cluster_))
          {
              *cluster_iter_input = *cluster_iter;
              cluster_iter_input++;
          }
      }

      clusters_ = std::vector<pcl::PointIndices> (clusters.begin (), cluster_iter_input);
      clusters.resize(clusters_.size());

      deinitCompute ();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   bool
      RegionGrowing::prepareForSegmentation ()
  {
      // if user forgot to pass point cloud or if it is empty
      if ( input_->points.size () == 0 )
          return (false);

      // if user forgot to pass normals or the sizes of point and normal cloud are different
      if ( normals_ == 0 || input_->points.size () != normals_->points.size () )
          return (false);

      // if residual test is on then we need to check if all needed parameters were correctly initialized
      if (residual_flag_)
      {
          if (residual_threshold_ <= 0.0f)
              return (false);
      }

      // if curvature test is on ...
      // if (curvature_flag_)
      // {
      //   in this case we do not need to check anything that related to it
      //   so we simply commented it
      // }

      // from here we check those parameters that are always valuable
      if (neighbour_radius_ == 0)
          return (false);

      // if user didn't set search method
      if (!search_)
          search_.reset (new pcl::search::KdTree<PointT>);

      if (indices_)
      {
          if (indices_->empty ())
              PCL_ERROR ("[pcl::RegionGrowing::prepareForSegmentation] Empty given indices!\n");
          search_->setInputCloud (input_, indices_);
      }
      else
          search_->setInputCloud (input_);

      return (true);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   void
      RegionGrowing::findPointNeighbours ()
  {
      int point_number = static_cast<int> (indices_->size ());
      std::vector<int> neighbours;
      std::vector<float> distances;

      point_neighbours_.resize (input_->points.size (), neighbours);
      if (input_->is_dense)
      {
          for (int i_point = 0; i_point < point_number; i_point++)
          {
              int point_index = (*indices_)[i_point];
              neighbours.clear ();
              search_->radiusSearch(i_point,neighbour_radius_,neighbours,distances);
              //search_->nearestKSearch (i_point, neighbour_number_, neighbours, distances);
              point_neighbours_[point_index].swap (neighbours);
          }
      }
      else
      {
          for (int i_point = 0; i_point < point_number; i_point++)
          {
              neighbours.clear ();
              int point_index = (*indices_)[i_point];
              if (!pcl::isFinite (input_->points[point_index]))
                  continue;
              //search_->nearestKSearch (i_point, neighbour_number_, neighbours, distances);
              search_->radiusSearch(i_point,neighbour_radius_,neighbours,distances);
              point_neighbours_[point_index].swap (neighbours);
          }
      }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   void
      RegionGrowing::applySmoothRegionGrowingAlgorithm ()
  {
      int num_of_pts = static_cast<int> (indices_->size ());
      point_labels_.resize (input_->points.size (), -1);

      std::vector< std::pair<float, int> > point_residual;
      std::pair<float, int> pair;
      point_residual.resize (num_of_pts, pair);

      if (normal_flag_ == true)
      {
          for (int i_point = 0; i_point < num_of_pts; i_point++)
          {
              int point_index = (*indices_)[i_point];
              point_residual[i_point].first = normals_->points[point_index].curvature;
              point_residual[i_point].second = point_index;
          }
          
          std::sort (point_residual.begin (), point_residual.end (), comparePair);
      }
      else
      {
          for (int i_point = 0; i_point < num_of_pts; i_point++)
          {
              int point_index = (*indices_)[i_point];
              point_residual[i_point].first = 0;
              point_residual[i_point].second = point_index;
          }
      }
      int seed_counter = 0;
      int seed = point_residual[seed_counter].second;

      int segmented_pts_num = 0;
      int number_of_segments = 0;
      while (segmented_pts_num < num_of_pts)
      {
          int pts_in_segment;
          pts_in_segment = growRegion (seed, number_of_segments);
          segmented_pts_num += pts_in_segment;
          num_pts_in_segment_.push_back (pts_in_segment);
          number_of_segments++;

          //find next point that is not segmented yet
          for (int i_seed = seed_counter + 1; i_seed < num_of_pts; i_seed++)
          {
              int index = point_residual[i_seed].second;
              if (point_labels_[index] == -1)
              {
                  seed = index;
                  break;
              }
          }
      }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   int
      RegionGrowing::growRegion (int initial_seed, int segment_number)
  {
      std::queue<int> seeds;
      seeds.push (initial_seed);
      point_labels_[initial_seed] = segment_number;

      int num_pts_in_segment = 1;

      while (!seeds.empty ())
      {
          int curr_seed;
          curr_seed = seeds.front ();
          seeds.pop ();

          size_t i_nghbr = 0;
          while ( /*i_nghbr < neighbour_number_&&*/ i_nghbr < point_neighbours_[curr_seed].size () )
          {
              int index = point_neighbours_[curr_seed][i_nghbr];//当前种子点的
              if (point_labels_[index] != -1)
              {
                  i_nghbr++;
                  continue;
              }

              bool is_a_seed = false;
              bool belongs_to_segment = validatePoint (initial_seed, curr_seed, index, is_a_seed);

              if (belongs_to_segment == false)
              {
                  i_nghbr++;
                  continue;
              }

              point_labels_[index] = segment_number;
              num_pts_in_segment++;

              if (is_a_seed)
              {
                  seeds.push (index);
              }

              i_nghbr++;
          }// next neighbour
      }// next seed

      return (num_pts_in_segment);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   bool
      RegionGrowing<PointT, NormalT>::validatePoint (int initial_seed, int point, int nghbr, bool& is_a_seed) const
  {
      is_a_seed = true;

      float cosine_threshold = cosf (theta_threshold_);
      float data[4];

      data[0] = input_->points[initial_seed].data[0];
      data[1] = input_->points[initial_seed].data[1];
      data[2] = input_->points[initial_seed].data[2];
      data[3] = input_->points[initial_seed].data[3];
      Eigen::Map<Eigen::Vector3f> initial_point (static_cast<float*> (data));
      Eigen::Map<Eigen::Vector3f> initial_normal (static_cast<float*> (normals_->points[initial_seed].normal));

      //check the angle between normals
      if (smooth_mode_flag_ == true)
      {
          Eigen::Map<Eigen::Vector3f> nghbr_normal (static_cast<float*> (normals_->points[nghbr].normal));
          float dot_product = fabsf (nghbr_normal.dot (initial_normal));
          if (dot_product < cosine_threshold)
          {
              is_a_seed = false;
              return (false);
          }
      }
      else
      {
          Eigen::Map<Eigen::Vector3f> nghbr_normal (static_cast<float*> (normals_->points[nghbr].normal));
          Eigen::Map<Eigen::Vector3f> initial_seed_normal (static_cast<float*> (normals_->points[initial_seed].normal));
          float dot_product = fabsf (nghbr_normal.dot (initial_seed_normal));
          if (dot_product < cosine_threshold)
              return (false);
      }

      // check the curvature if needed
      if (curvature_flag_ && normals_->points[nghbr].curvature > curvature_threshold_)
      {
          is_a_seed = false;
          return (false);
      }
      float data_1[4];

      // check the residual if needed
      data_1[0] = input_->points[nghbr].data[0];
      data_1[1] = input_->points[nghbr].data[1];
      data_1[2] = input_->points[nghbr].data[2];
      data_1[3] = input_->points[nghbr].data[3];
      Eigen::Map<Eigen::Vector3f> nghbr_point (static_cast<float*> (data_1));
      float residual = fabsf (initial_normal.dot (initial_point - nghbr_point));
      if (residual_flag_ && residual > residual_threshold_)
      {
          is_a_seed = false;
          return (false);
      }

      return (true);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   void
      RegionGrowing<PointT, NormalT>::assembleRegions ()
  {
      int number_of_segments = static_cast<int> (num_pts_in_segment_.size ());
      int number_of_points = static_cast<int> (input_->points.size ());

      pcl::PointIndices segment;
      clusters_.resize (number_of_segments, segment);

      for (int i_seg = 0; i_seg < number_of_segments; i_seg++)
      {
          clusters_[i_seg].indices.resize ( num_pts_in_segment_[i_seg], 0);
      }

      std::vector<int> counter;
      counter.resize (number_of_segments, 0);

      for (int i_point = 0; i_point < number_of_points; i_point++)
      {
          int segment_index = point_labels_[i_point];
          if (segment_index != -1)
          {
              int point_index = counter[segment_index];
              clusters_[segment_index].indices[point_index] = i_point;
              counter[segment_index] = point_index + 1;
          }
      }

      number_of_segments_ = number_of_segments;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   void
      RegionGrowing<PointT, NormalT>::getPlaneFromPoint(vector<PlanSegment>& planes,const vector<pcl::PointIndices>& clusters)
   {
       for (int i=0;i<clusters.size();i++)
       {
           PlanSegment plane;
           plane.points_id=clusters[i];
           CloudPtr cloud(new Cloud);
           for (int j=0;j<clusters[i].indices.size();j++)
           {
               CloudItem temp=input_->points[clusters[i].indices[j]];
               cloud->push_back(temp);
           }
//            srand((unsigned)time(NULL));
//            boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("LineSegmentDisplay"));
//            viewer->setBackgroundColor(0,0,0);
           FeatureCalculate::rpca_plane(cloud,plane,0.99,0.5,0.5);
//            pcl::ModelCoefficients coeffs;
//            coeffs.values.push_back(plane.normal.normal_x);
//            coeffs.values.push_back(plane.normal.normal_y);
//            coeffs.values.push_back(plane.normal.normal_z);
//            coeffs.values.push_back(plane.distance);
//            viewer->addPlane (coeffs, "line");
//            viewer->addPointCloud(cloud,"cloud");
//            while (!viewer->wasStopped ())
//            {
//                viewer->spinOnce();
//                boost::this_thread::sleep (boost::posix_time::microseconds (5));
//            }
           CloudItem minPt, maxPt;
           pcl::getMinMax3D (*cloud,minPt,maxPt);
           float delta_z=maxPt.z-minPt.z;   //墙高达到0.8米才判定为墙
           if (plane.normal.normal_z<0.5&&plane.normal.curvature<0.1&&plane.points_id.indices.size()>25&&delta_z>0.8)
           {
               planes.push_back(plane);
           }
       }

   }

   void
      RegionGrowing<PointT, NormalT>::getSegmentFromPoint (int index, pcl::PointIndices& cluster)
  {
      cluster.indices.clear ();

      bool segmentation_is_possible = initCompute ();
      if ( !segmentation_is_possible )
      {
          deinitCompute ();
          return;
      }

      // first of all we need to find out if this point belongs to cloud
      bool point_was_found = false;
      int number_of_points = static_cast <int> (indices_->size ());
      for (int point = 0; point < number_of_points; point++)
          if ( (*indices_)[point] == index)
          {
              point_was_found = true;
              break;
          }

          if (point_was_found)
          {
              if (clusters_.empty ())
              {
                  point_neighbours_.clear ();
                  point_labels_.clear ();
                  num_pts_in_segment_.clear ();
                  number_of_segments_ = 0;

                  segmentation_is_possible = prepareForSegmentation ();
                  if ( !segmentation_is_possible )
                  {
                      deinitCompute ();
                      return;
                  }

                  findPointNeighbours ();
                  applySmoothRegionGrowingAlgorithm ();
                  assembleRegions ();
              }
              // if we have already made the segmentation, then find the segment
              // to which this point belongs
              std::vector <pcl::PointIndices>::iterator i_segment;
              for (i_segment = clusters_.begin (); i_segment != clusters_.end (); i_segment++)
              {
                  bool segment_was_found = false;
                  for (size_t i_point = 0; i_point < i_segment->indices.size (); i_point++)
                  {
                      if (i_segment->indices[i_point] == index)
                      {
                          segment_was_found = true;
                          cluster.indices.clear ();
                          cluster.indices.reserve (i_segment->indices.size ());
                          std::copy (i_segment->indices.begin (), i_segment->indices.end (), std::back_inserter (cluster.indices));
                          break;
                      }
                  }
                  if (segment_was_found)
                  {
                      break;
                  }
              }// next segment
          }// end if point was found

          deinitCompute ();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      RegionGrowing<PointT, NormalT>::getColoredCloud ()
  {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;

      if (!clusters_.empty ())
      {
          colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared ();

          srand (static_cast<unsigned int> (time (0)));
          std::vector<unsigned char> colors;
          for (size_t i_segment = 0; i_segment < clusters_.size (); i_segment++)
          {
              colors.push_back (static_cast<unsigned char> (rand () % 256));
              colors.push_back (static_cast<unsigned char> (rand () % 256));
              colors.push_back (static_cast<unsigned char> (rand () % 256));
          }

          colored_cloud->width = input_->width;
          colored_cloud->height = input_->height;
          colored_cloud->is_dense = input_->is_dense;
          for (size_t i_point = 0; i_point < input_->points.size (); i_point++)
          {
              pcl::PointXYZRGB point;
              point.x = *(input_->points[i_point].data);
              point.y = *(input_->points[i_point].data + 1);
              point.z = *(input_->points[i_point].data + 2);
              point.r = 0;
              point.g = 0;
              point.b = 0;
              colored_cloud->points.push_back (point);
          }

          std::vector< pcl::PointIndices >::iterator i_segment;
          int next_color = 0;
          for (i_segment = clusters_.begin (); i_segment != clusters_.end (); i_segment++)
          {
              PlanSegment i_segment_plane;
              getPlaneFromPoint(i_segment_plane,*i_segment);
              if(i_segment_plane.normal.normal_z<0.1)
              {
              std::vector<int>::iterator i_point;
              for (i_point = i_segment->indices.begin (); i_point != i_segment->indices.end (); i_point++)
              {
                  int index;
                  index = *i_point;
                  colored_cloud->points[index].r = colors[3 * next_color];
                  colored_cloud->points[index].g = colors[3 * next_color + 1];
                  colored_cloud->points[index].b = colors[3 * next_color + 2];
              }
              next_color++;
              }
              else
              {

              }
          }
      }

      return (colored_cloud);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
      RegionGrowing<PointT, NormalT>::getColoredCloudRGBA ()
  {
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr colored_cloud;

      if (!clusters_.empty ())
      {
          colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGBA>)->makeShared ();

          srand (static_cast<unsigned int> (time (0)));
          std::vector<unsigned char> colors;
          for (size_t i_segment = 0; i_segment < clusters_.size (); i_segment++)
          {
              colors.push_back (static_cast<unsigned char> (rand () % 256));
              colors.push_back (static_cast<unsigned char> (rand () % 256));
              colors.push_back (static_cast<unsigned char> (rand () % 256));
          }

          colored_cloud->width = input_->width;
          colored_cloud->height = input_->height;
          colored_cloud->is_dense = input_->is_dense;
          for (size_t i_point = 0; i_point < input_->points.size (); i_point++)
          {
              pcl::PointXYZRGBA point;
              point.x = *(input_->points[i_point].data);
              point.y = *(input_->points[i_point].data + 1);
              point.z = *(input_->points[i_point].data + 2);
              point.r = 255;
              point.g = 0;
              point.b = 0;
              point.a = 0;
              colored_cloud->points.push_back (point);
          }

          std::vector< pcl::PointIndices >::iterator i_segment;
          int next_color = 0;
          for (i_segment = clusters_.begin (); i_segment != clusters_.end (); i_segment++)
          {
              PlanSegment i_segment_plane;
              getPlaneFromPoint(i_segment_plane,*i_segment);
              if(i_segment_plane.normal.normal_z<0.1)
              {
                  std::vector<int>::iterator i_point;
                  for (i_point = i_segment->indices.begin (); i_point != i_segment->indices.end (); i_point++)
                  {
                      int index;
                      index = *i_point;
                      colored_cloud->points[index].r = colors[3 * next_color];
                      colored_cloud->points[index].g = colors[3 * next_color + 1];
                      colored_cloud->points[index].b = colors[3 * next_color + 2];
                  }
                  next_color++;
              }
              else
              {

              }
          }
      }

      return (colored_cloud);
  }


  };