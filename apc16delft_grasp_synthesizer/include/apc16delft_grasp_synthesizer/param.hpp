#pragma once

#include <dr_eigen/param.hpp>
#include <dr_param/param.hpp>
#include <dr_eigen/eigen.hpp>
#include <dr_eigen/ros.hpp>
#include <dr_param/xmlrpc.hpp>

#include <apc16delft_msgs/GraspCandidate.h>
#include <apc16delft_util/util.hpp>
#include "grasp_item.hpp"

using namespace apc16delft;
using namespace apc16delft_msgs;

namespace dr {

/// Load additional candidates.
void loadAdditionalCandidates(std::vector<GraspCandidate> & original, XmlRpc::XmlRpcValue const & value) {
		std::vector<GraspCandidate> candidates = fromXmlRpc<std::vector<GraspCandidate>>(value);
		original.insert(std::end(original), std::begin(candidates), std::end(candidates));
};

/// Convert an XmlRpcValue to a GraspCandidate.
template<> GraspCandidate fromXmlRpc<GraspCandidate>(XmlRpc::XmlRpcValue const & value) {
	ensureXmlRpcType(value, XmlRpc::XmlRpcValue::TypeStruct, "GraspCandidate");

	GraspCandidate grasp_candidate;

	grasp_candidate.stamped_pose.pose = dr::toRosPose(fromXmlRpc<Eigen::Isometry3d>(xmlRpcAt(value, "pose")));
	grasp_candidate.score             = fromXmlRpc<double>(xmlRpcAt(value, "score"));

	return grasp_candidate;
};

/// Convert an XmlRpcValue to a GraspItem.
template<> GraspItem fromXmlRpc<GraspItem>(XmlRpc::XmlRpcValue const & value) {
	ensureXmlRpcType(value, XmlRpc::XmlRpcValue::TypeStruct, "GraspItem");
	XmlRpc::XmlRpcValue center_of_mass = xmlRpcAt(value, "CoM");

	GraspItem grasp_item;
	grasp_item.pre_grasp_offset = fromXmlRpc<double>(xmlRpcAt(value, "pre_grasp_offset"));
	grasp_item.center_of_mass   = fromXmlRpc<Eigen::Vector3d>(xmlRpcAt(center_of_mass, "position"));
	grasp_item.sample_spaces    = fromXmlRpc<std::vector<Shape>>(xmlRpcAt(value, "sample_space"));
	grasp_item.model_path       = resolvePackagePath(fromXmlRpc<std::string>(xmlRpcAt(value, "model_path")));

	return grasp_item;
};

/// Convert an XmlRpcValue to a deformable object.
template<> Deformable fromXmlRpc<Deformable>(XmlRpc::XmlRpcValue const & value) {
	ensureXmlRpcType(value, XmlRpc::XmlRpcValue::TypeStruct, "Deformable");

	Deformable deformable;
	if (value.hasMember("extra_candidates")) {
		loadAdditionalCandidates(deformable.grasp_candidates, xmlRpcAt(value, "extra_candidates"));
	}

	return deformable;
};

/// Convert an XmlRpcValue to a deformable object.
template<> Manual fromXmlRpc<Manual>(XmlRpc::XmlRpcValue const & value) {
	ensureXmlRpcType(value, XmlRpc::XmlRpcValue::TypeStruct, "Manual");

	Manual manual;
	if (value.hasMember("extra_candidates")) {
		loadAdditionalCandidates(manual.grasp_candidates, xmlRpcAt(value, "extra_candidates"));
	}

	return manual;
};

/// Convert an XmlRpcValue to a ring.
template<> Ring fromXmlRpc<Ring>(XmlRpc::XmlRpcValue const & value) {
	ensureXmlRpcType(value, XmlRpc::XmlRpcValue::TypeStruct, "Ring");

	XmlRpc::XmlRpcValue dimensions = xmlRpcAt(value, "dimensions");

	Eigen::Isometry3d origin      = fromXmlRpc<Eigen::Isometry3d>(xmlRpcAt(value , "origin"                 ));
	double inner_radius           = fromXmlRpc<double>(xmlRpcAt(dimensions       , "inner_radius"           ));
	double outer_radius           = fromXmlRpc<double>(xmlRpcAt(dimensions       , "outer_radius"           ));
	double height                 = fromXmlRpc<double>(xmlRpcAt(dimensions       , "height"                 ));
	double distance               = fromXmlRpc<double>(xmlRpcAt(value            , "point_distance"         ));
	double intersection_threshold = fromXmlRpc<double>(xmlRpcAt(value            , "intersection_threshold" ));
	double edge_clearing          = fromXmlRpc<double>(xmlRpcAt(value            , "edge_clearing"          ));
	bool  vacuum                  = fromXmlRpc<bool>  (xmlRpcAt(value            , "vacuum"                 ));
	bool generate                 = fromXmlRpc<bool>  (xmlRpcAt(value            , "generate"               ));

	Ring ring{inner_radius, outer_radius, height, distance, intersection_threshold, edge_clearing, vacuum, generate, origin};

	if (value.hasMember("extra_candidates")) {
		loadAdditionalCandidates(ring.grasp_candidates, xmlRpcAt(value, "extra_candidates"));
	}

	return ring;
};

/// Convert an XmlRpcValue to a Bar.
template<> Bar fromXmlRpc<Bar>(XmlRpc::XmlRpcValue const & value) {
	ensureXmlRpcType(value, XmlRpc::XmlRpcValue::TypeStruct, "Bar");

	Eigen::Vector3d dimensions    = fromXmlRpc<Eigen::Vector3d>(xmlRpcAt(value   , "dimensions"             ));
	Eigen::Isometry3d origin      = fromXmlRpc<Eigen::Isometry3d>(xmlRpcAt(value , "origin"                 ));
	double distance               = fromXmlRpc<double>(xmlRpcAt(value            , "point_distance"         ));
	double edge_clearing          = fromXmlRpc<double>(xmlRpcAt(value            , "edge_clearing"          ));
	double intersection_threshold = fromXmlRpc<double>(xmlRpcAt(value            , "intersection_threshold" ));
	bool vacuum                   = fromXmlRpc<bool>(xmlRpcAt(value              , "vacuum"                 ));
	bool generate                 = fromXmlRpc<bool>  (xmlRpcAt(value            , "generate"               ));

	Bar bar{dimensions, origin, distance, intersection_threshold, edge_clearing, vacuum, generate};

	if (value.hasMember("extra_candidates")) {
		loadAdditionalCandidates(bar.grasp_candidates, xmlRpcAt(value, "extra_candidates"));
	}

	return bar;
};

/// Convert an XmlRpcValue to a Cylinder.
template<> Cylinder fromXmlRpc<Cylinder>(XmlRpc::XmlRpcValue const & value) {
	ensureXmlRpcType(value, XmlRpc::XmlRpcValue::TypeStruct, "Cylinder");
	XmlRpc::XmlRpcValue dimensions = xmlRpcAt(value, "dimensions");

	Eigen::Isometry3d origin      = fromXmlRpc<Eigen::Isometry3d>(xmlRpcAt(value , "origin"                 ));
	double radius                 = fromXmlRpc<double>(xmlRpcAt(dimensions       , "radius"                 ));
	double height                 = fromXmlRpc<double>(xmlRpcAt(dimensions       , "height"                 ));
	double distance               = fromXmlRpc<double>(xmlRpcAt(value            , "point_distance"         ));
	double intersection_threshold = fromXmlRpc<double>(xmlRpcAt(value            , "intersection_threshold" ));
	double edge_clearing          = fromXmlRpc<double>(xmlRpcAt(value            , "edge_clearing"          ));
	bool vacuum                   = fromXmlRpc<bool>(xmlRpcAt(value              , "vacuum"                 ));
	bool generate                 = fromXmlRpc<bool>  (xmlRpcAt(value            , "generate"               ));

	Cylinder cylinder{radius, height, distance, intersection_threshold, edge_clearing, vacuum, generate,origin};

	if (value.hasMember("extra_candidates")) {
		loadAdditionalCandidates(cylinder.grasp_candidates, xmlRpcAt(value, "extra_candidates"));
	}

	return cylinder;
};

/// Convert an XmlRpcValue to a Cone.
template<> Cone fromXmlRpc<Cone>(XmlRpc::XmlRpcValue const & value) {
	ensureXmlRpcType(value, XmlRpc::XmlRpcValue::TypeStruct, "Cone");
	XmlRpc::XmlRpcValue dimensions = xmlRpcAt(value, "dimensions");

	Eigen::Isometry3d origin      = fromXmlRpc<Eigen::Isometry3d>(xmlRpcAt(value , "origin"                 ));
	double radius                 = fromXmlRpc<double>(xmlRpcAt(dimensions       , "radius"                 ));
	double height                 = fromXmlRpc<double>(xmlRpcAt(dimensions       , "height"                 ));
	double distance               = fromXmlRpc<double>(xmlRpcAt(value            , "point_distance"         ));
	double intersection_threshold = fromXmlRpc<double>(xmlRpcAt(value            , "intersection_threshold" ));
	bool vacuum                   = fromXmlRpc<bool>(xmlRpcAt(value              , "vacuum"                 ));
	bool generate                 = fromXmlRpc<bool>  (xmlRpcAt(value            , "generate"               ));

	Cone cone{radius, height, distance, intersection_threshold, vacuum, generate,origin};

	if (value.hasMember("extra_candidates")) {
		loadAdditionalCandidates(cone.grasp_candidates, xmlRpcAt(value, "extra_candidates"));
	}

	return cone;
};

/// Convert an XmlRpcValue to a Sphere.
template<> Sphere fromXmlRpc<Sphere>(XmlRpc::XmlRpcValue const & value) {
	ensureXmlRpcType(value, XmlRpc::XmlRpcValue::TypeStruct, "Sphere");

	Eigen::Isometry3d origin      = fromXmlRpc<Eigen::Isometry3d>(xmlRpcAt(value , "origin"                 ));
	double radius                 = fromXmlRpc<double>(xmlRpcAt(value            , "radius"                 ));
	double distance               = fromXmlRpc<double>(xmlRpcAt(value            , "point_distance"         ));
	double intersection_threshold = fromXmlRpc<double>(xmlRpcAt(value            , "intersection_threshold" ));
	bool vacuum                   = fromXmlRpc<bool>(xmlRpcAt(value              , "vacuum"                 ));
	bool generate                 = fromXmlRpc<bool>  (xmlRpcAt(value            , "generate"               ));

	Sphere sphere{radius, distance, intersection_threshold, vacuum, generate,origin};

	if (value.hasMember("extra_candidates")) {
		loadAdditionalCandidates(sphere.grasp_candidates, xmlRpcAt(value, "extra_candidates"));
	}

	return sphere;
};

/// Convert an XmlRpcValue to a Circle.
template<> Circle fromXmlRpc<Circle>(XmlRpc::XmlRpcValue const & value) {
	ensureXmlRpcType(value, XmlRpc::XmlRpcValue::TypeStruct, "Circle");

	Eigen::Isometry3d origin      = fromXmlRpc<Eigen::Isometry3d>(xmlRpcAt(value , "origin"                 ));
	double radius                 = fromXmlRpc<double>(xmlRpcAt(value            , "radius"                 ));
	double distance               = fromXmlRpc<double>(xmlRpcAt(value            , "point_distance"         ));
	double intersection_threshold = fromXmlRpc<double>(xmlRpcAt(value            , "intersection_threshold" ));
	bool vacuum                   = fromXmlRpc<bool>(xmlRpcAt(value              , "vacuum"                 ));
	bool generate                 = fromXmlRpc<bool>  (xmlRpcAt(value            , "generate"               ));
	int strategy                  = fromXmlRpc<int>(xmlRpcAt(value               , "strategy"               ));
	int direction                 = fromXmlRpc<int>(xmlRpcAt(value               , "direction"              ));

	Circle circle{radius, distance, intersection_threshold, direction, vacuum, generate, origin, strategy};

	if (value.hasMember("extra_candidates")) {
		loadAdditionalCandidates(circle.grasp_candidates, xmlRpcAt(value, "extra_candidates"));
	}

	return circle;
};

/// Convdrt an XmlRpcValue to a shape.
template<> Shape fromXmlRpc<Shape>(XmlRpc::XmlRpcValue const & value) {
	ensureXmlRpcType(value, XmlRpc::XmlRpcValue::TypeStruct, "Shape");

	std::string type = fromXmlRpc<std::string>(xmlRpcAt(value, "shape"));

	if (type.compare("bar") == 0)
		return fromXmlRpc<Bar>(value);
	else if (type.compare("sphere") == 0)
		return fromXmlRpc<Sphere>(value);
	else if (type.compare("cone") == 0)
		return fromXmlRpc<Cone>(value);
	else if (type.compare("cylinder") == 0)
		return fromXmlRpc<Cylinder>(value);
	else if (type.compare("circle") == 0)
		return fromXmlRpc<Circle>(value);
	else if (type.compare("deformable") == 0)
		return fromXmlRpc<Deformable>(value);
	else if (type.compare("manual") == 0)
		return fromXmlRpc<Manual>(value);
	else if (type.compare("ring") == 0)
		return fromXmlRpc<Ring>(value);
	else
		throw std::runtime_error("Trying to load unknown shape: " + type);
};

}

