#ifndef VOXBLOX_ROS_CONVERSIONS_INL_H_
#define VOXBLOX_ROS_CONVERSIONS_INL_H_

#include <vector>

namespace voxblox {

template <typename VoxelType>
void serializeLayerAsMsg(const Layer<VoxelType>& layer, const bool only_updated,
                         voxblox_msgs::Layer* msg,
                         const MapDerializationAction& action) {
  double init_t = ros::Time::now().toSec();
  CHECK_NOTNULL(msg);
  msg->voxels_per_side = layer.voxels_per_side();
  msg->voxel_size = layer.voxel_size();

  msg->layer_type = getVoxelType<VoxelType>();

  BlockIndexList block_list;
  if (only_updated) {
    // layer.getAllUpdatedBlocks(Update::kMap, &block_list);
    layer.getAllUpdatedBlocks(Update::kSend, &block_list);
  } else {
    layer.getAllAllocatedBlocks(&block_list);
  }

  msg->action = static_cast<uint8_t>(action);

  voxblox_msgs::Block block_msg;
  msg->blocks.reserve(block_list.size());

  // New
  Layer<VoxelType>& layer_ = const_cast<Layer<VoxelType>&>(layer);
  // END New

  for (const BlockIndex& index : block_list) {

    // std::bitset<Update::kCount> tomeu = layer.getBlockByIndex(index).updated();
    // ROS_ERROR("%s map(%d, %d, %d) Updated: (%d%d%d%d)", getVoxelType<VoxelType>().c_str(), index.x(), index.y(), index.z(), 
    //                                       tomeu.test(Update::kSend) ? 1 : 0, tomeu.test(Update::kEsdf) ? 1 : 0,
    //                                       tomeu.test(Update::kMesh) ? 1 : 0,tomeu.test(Update::kMap) ? 1 : 0);

    // New
    layer_.getBlockPtrByIndex(index)->updated().reset(Update::kSend);
    // END New

    block_msg.x_index = index.x();
    block_msg.y_index = index.y();
    block_msg.z_index = index.z();

    std::vector<uint32_t> data;
    layer.getBlockByIndex(index).serializeToIntegers(&data);

    block_msg.data = data;
    msg->blocks.push_back(block_msg);
  }

  // ROS_WARN("%2.12lfs to serialize the %s layer with %d blocks", ros::Time::now().toSec() - init_t,
  //                                                         getVoxelType<VoxelType>().c_str(),
  //                                                         msg->blocks.size());

}  // namespace voxblox

template <typename VoxelType>
bool deserializeMsgToLayer(const voxblox_msgs::Layer& msg,
                           Layer<VoxelType>* layer) {
  CHECK_NOTNULL(layer);
  return deserializeMsgToLayer<VoxelType>(
      msg, static_cast<MapDerializationAction>(msg.action), layer);
}

template <typename VoxelType>
bool deserializeMsgToLayer(const voxblox_msgs::Layer& msg,
                           const MapDerializationAction& action,
                           Layer<VoxelType>* layer) {

  double init_t = ros::Time::now().toSec();
  CHECK_NOTNULL(layer);
  if (getVoxelType<VoxelType>().compare(msg.layer_type) != 0) {
    return false;
  }

  // So we also need to check if the sizes match. If they don't, we can't
  // parse this at all.
  constexpr double kVoxelSizeEpsilon = 1e-5;
  if (msg.voxels_per_side != layer->voxels_per_side() ||
      std::abs(msg.voxel_size - layer->voxel_size()) > kVoxelSizeEpsilon) {
    LOG(ERROR) << "Sizes don't match!";
    return false;
  }

  if (action == MapDerializationAction::kReset) {
    LOG(INFO) << "Resetting current layer.";
    layer->removeAllBlocks();
  }

  for (const voxblox_msgs::Block& block_msg : msg.blocks) {
    BlockIndex index(block_msg.x_index, block_msg.y_index, block_msg.z_index);

    // Either we want to update an existing block or there was no block there
    // before.
    if (action == MapDerializationAction::kUpdate || !layer->hasBlock(index)) {
      // Create a new block if it doesn't exist yet, or get the existing one
      // at the correct block index.
      typename Block<VoxelType>::Ptr block_ptr =
          layer->allocateBlockPtrByIndex(index);

      std::vector<uint32_t> data = block_msg.data;
      block_ptr->deserializeFromIntegers(data);

    } else if (action == MapDerializationAction::kMerge) {
      typename Block<VoxelType>::Ptr old_block_ptr =
          layer->getBlockPtrByIndex(index);
      CHECK(old_block_ptr);

      typename Block<VoxelType>::Ptr new_block_ptr(new Block<VoxelType>(
          old_block_ptr->voxels_per_side(), old_block_ptr->voxel_size(),
          old_block_ptr->origin()));

      std::vector<uint32_t> data = block_msg.data;
      new_block_ptr->deserializeFromIntegers(data);

      old_block_ptr->mergeBlock(*new_block_ptr);
    }

    // Llevar
    // if (getVoxelType<TsdfVoxel>().compare(msg.layer_type) != 0) {
    //   typename Block<VoxelType>::Ptr block_ptr_ = layer->getBlockPtrByIndex(index);

    //   // const size_t voxels_per_side = block_ptr->voxels_per_side();
    //   // const size_t voxels_per_block = voxels_per_side * voxels_per_side * voxels_per_side;

    //   // Loop through all voxels in the block
    //   for (size_t z = 0; z < msg.voxels_per_side; ++z) {
    //       for (size_t y = 0; y < msg.voxels_per_side; ++y) {
    //           for (size_t x = 0; x < msg.voxels_per_side; ++x) {
    //               // Create a voxel index
    //               voxblox::VoxelIndex voxel_index(x, y, z);

    //               // Get the voxel position in world coordinates
    //               voxblox::Point voxel_position = block_ptr_->computeCoordinatesFromVoxelIndex(voxel_index);


    //               // Now you can use voxel_position as needed
    //               // For example, you can access its x, y, z coordinates as follows:
    //               double x_ = voxel_position.x();
    //               double y_ = voxel_position.y();
    //               double z_ = voxel_position.z();
    //               ROS_WARN("X: %2.3lf, Y: %2.3lf, Z: %2.3lf", x_, y_, z_);
    //           }
    //       }
    //   }
    // }
    // Llevar

  }


  switch (action) {
    case MapDerializationAction::kReset:
      CHECK_EQ(layer->getNumberOfAllocatedBlocks(), msg.blocks.size());
      break;
    case MapDerializationAction::kUpdate:
    // Fall through intended.
    case MapDerializationAction::kMerge:
      CHECK_GE(layer->getNumberOfAllocatedBlocks(), msg.blocks.size());
      break;
  }

  // ROS_WARN("%2.12lfs to deserialize the %s layer with %d blocks", ros::Time::now().toSec() - init_t,
  //                                                         getVoxelType<VoxelType>().c_str(),
  //                                                         msg.blocks.size());

  return true;
}

}  // namespace voxblox

#endif  // VOXBLOX_ROS_CONVERSIONS_INL_H_
