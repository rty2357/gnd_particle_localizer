/*
 * gnd_particle_localizer_config.hpp
 */

#ifndef GND_PARTICLE_LOCALIZER_CONFIG_HPP_
#define GND_PARTICLE_LOCALIZER_CONFIG_HPP_

#include <string.h>

#include "gnd/gnd-util.h"
#include "gnd/gnd-config-file.hpp"
#include "gnd/gnd-lib-error.h"

// ---> type declaration
namespace gnd {
	namespace particle_localizer {
		struct node_config;
		typedef struct node_config node_config;

		typedef gnd::conf::parameter_array<char, 256> param_string_t;
		typedef gnd::conf::parameter_array<double, 3> param_pose_t;
		typedef gnd::conf::parameter_array<double, 9> param_pose_covariance_t;
		typedef gnd::conf::param_int param_int_t;
		typedef gnd::conf::param_long param_long_t;
		typedef gnd::conf::param_double param_double_t;
		typedef gnd::conf::param_bool param_bool_t;
	}
} // <--- type declaration


// ---> const variables definition
namespace gnd {
	namespace particle_localizer {

		// ---> ros communication
		static const param_string_t Default_node_name = {
				"node-name",
				"particle_localizer",
				"ros-node name"
		};

		static const param_string_t Default_topic_name_pose = {
				"topic-pose",
				"pose",
				"estimated pose topic name, (publish, type:gnd_geometry2d_msgs/msg_pose2d_stamped)"
		};


		static const param_string_t Default_topic_name_particles = {
				"topic-particles",
				"localization_particles",
				"particles topic name, (publish, type:gnd_particle_localiser/msg_pose2darray_stamped)"
		};

		static const param_string_t Default_topic_name_particle_weight = {
				"topic-particle-weights",
				"localization_particles/weights",
				"particle weights topic name, (subscribe, gnd_particle_localiser/msg_localization_particle_weight)"
		};

		static const param_string_t Default_topic_name_motion = {
				"topic-motion",
				"velocity",
				"motion model topic name, (subscribe, type:gnd_geometry2d_msgs/msg_velocity2d_with_covariance_stamped)"
		};

		static const param_string_t Default_service_name_reset_particles_nd = {
				"service-reset-particles-normal-distribution",
				"reset_particles_nd",
				"service name, (gnd_particle_localizer/srv_reset_particles_normal_distribution)"
		};

		// <--- ros communication

		// ---> initial parameter
		static const param_pose_t Default_initial_pose = {
				"initial-pose",
				{0.0, 0.0, gnd_ang2deg(0.0)},
				"initial pose"
		};

		static const param_pose_covariance_t Default_error_covariance_initial_pose = {
				"standard_error_initial-pose",
				{0.3 * 0.3, 0.0,       0.0,
				 0.0,       0.3 * 0.3, 0.0,
				 0.0,       0.0,       gnd_deg2ang(10.0) * gnd_deg2ang(10.0)},
				"standard error of initial pose"
		};
		// <--- initial parameter

		// ---> particle filter parameter
		static const param_int_t Default_number_of_particles= {
				"number-of-particles",
				100,
				"the number of particles"
		};

		static const param_double_t Default_period_resampling = {
				"period-resampling",
				gnd_sec2time(1.0),
				"period to resampling"
		};
		// <--- particle filter parameter



		// ---> debug option
		static const param_double_t Default_period_cui_status_display = {
				"period-status-display",
				0.0,
				"period to display the node status in standard output [msec]. [note] it need ansi color code. [note] if this value is less than or equal 0, don't display"
		};

		static const param_string_t Default_particles_log = {
				"localization-log-txt",
				"",
				"localization log(text file)"
		};
		// <--- debug option
	}
}
// <--- const variables definition



// ---> function declaration
namespace gnd {
	namespace particle_localizer {

		/**
		 * @brief initialize configure to default parameter
		 * @param [out] p : node_config
		 */
		int init_node_config(node_config *conf);


		/**
		 * @brief config file read
		 * @param [in] fname : file name
		 * @param [out] dest : configuration parameter
		 */
		int fread_node_config( const char* fname, node_config *dest );
		/**
		 * @brief get config parameter from description
		 * @param [out] dest : configuration parameter
		 * @param [in]  src  : configuration description
		 */
		int get_node_config( node_config *dest, gnd::conf::configuration *src );



		/**
		 * @brief config file write
		 * @param [in] fname : file name
		 * @param [in] src   : configuration parameter
		 */
		int fwrite_node_config( const char* fname, node_config *src );
		/**
		 * @brief set config description
		 * @param [out] dest : description
		 * @param [in]   src : parameter
		 */
		int set_node_config( gnd::conf::configuration *dest, node_config *src );
	}
} // <--- function declaration



// ---> type definition
namespace gnd {
	namespace particle_localizer {
		/**
		 * @brief configuration parameter for gnd_urg_proxy node
		 */
		struct node_config {
			node_config();

			// ros communication
			param_string_t node_name;								///< ros-node name
			param_string_t topic_name_pose;							///< estimated pose topic name
			param_string_t topic_name_particles;					///< particles topic name
			param_string_t topic_name_particle_weights;				///< particle weights topic name
			param_string_t topic_name_motion;						///< motion model topic name
			param_string_t service_name_reset_particles_nd;			///< reset particle service name
			// initial position option
			param_pose_t initial_pose;								///< mean of estimated initial pose
			param_pose_covariance_t error_covariance_initial_pose;	///< error co-variance of initial pose
			// particle filter parameter
			param_int_t number_of_particles;						///< the number of particles
			param_double_t period_resampling;						///< cycle to resampling
			// debug option
			param_double_t period_cui_status_display;				///< no device mode for debug
			param_string_t particles_log;							///< particles log file
		};


		inline
		node_config::node_config() {
			init_node_config(this);
		}
	}
}
// <--- type definition



// ---> function definition
namespace gnd {
	namespace particle_localizer {
		/*
		 * @brief initialize configuration parameter
		 * @param [out] p : node_config
		 */
		inline
		int init_node_config( node_config *p ){
			gnd_assert(!p, -1, "invalid null pointer argument\n" );

			// ros communication parameter
			memcpy( &p->node_name,						&Default_node_name,							sizeof(Default_node_name) );
			memcpy( &p->topic_name_pose,				&Default_topic_name_pose,					sizeof(Default_topic_name_pose) );
			memcpy( &p->topic_name_particles,			&Default_topic_name_particles,				sizeof(Default_topic_name_particles) );
			memcpy( &p->topic_name_particle_weights,	&Default_topic_name_particle_weight,		sizeof(Default_topic_name_particle_weight) );
			memcpy( &p->topic_name_motion,				&Default_topic_name_motion,					sizeof(Default_topic_name_motion) );
			memcpy( &p->service_name_reset_particles_nd,&Default_service_name_reset_particles_nd,	sizeof(Default_service_name_reset_particles_nd) );
			// initial pose option
			memcpy( &p->initial_pose,					&Default_initial_pose,						sizeof(Default_initial_pose) );
			memcpy( &p->error_covariance_initial_pose,	&Default_error_covariance_initial_pose,		sizeof(Default_error_covariance_initial_pose) );
			// particle filter parameter
			memcpy( &p->number_of_particles,			&Default_number_of_particles,				sizeof(Default_number_of_particles) );
			memcpy( &p->period_resampling,				&Default_period_resampling,					sizeof(Default_period_resampling) );
			// debug option
			memcpy( &p->period_cui_status_display,		&Default_period_cui_status_display,			sizeof(Default_period_cui_status_display) );
			memcpy( &p->particles_log,					&Default_particles_log,						sizeof(Default_particles_log) );
			return 0;
		}

		/*
		 * @brief config file read
		 * @param [in] fname : file name
		 * @param [out] dest : configuration parameter
		 */
		inline
		int fread_node_config( const char* fname, node_config *dest ) {
			gnd_assert(!fname, -1, "invalid null pointer argument\n" );
			gnd_assert(!dest, -1, "invalid null pointer argument\n" );

			{ // ---> operation
				int ret;
				gnd::conf::file_stream fs;
				// configuration file read
				if( (ret = fs.read(fname)) < 0 )    return ret;

				return get_node_config(dest, &fs);
			} // <--- operation
		}
		/*
		 * @brief get config parameter from description
		 * @param [out] dest : configuration parameter
		 * @param [in]  src  : configuration description
		 */
		inline
		int get_node_config( node_config *dest, gnd::conf::configuration *src ) {
			gnd_assert(!dest, -1, "invalid null pointer argument\n" );
			gnd_assert(!src, -1, "invalid null pointer argument\n" );

			// ros communication parameter
			gnd::conf::get_parameter( src, &dest->node_name );
			gnd::conf::get_parameter( src, &dest->topic_name_pose );
			gnd::conf::get_parameter( src, &dest->topic_name_particles );
			gnd::conf::get_parameter( src, &dest->topic_name_particle_weights );
			gnd::conf::get_parameter( src, &dest->topic_name_motion );
			gnd::conf::get_parameter( src, &dest->service_name_reset_particles_nd );
			// initial pose option
			gnd::conf::get_parameter( src, &dest->initial_pose );
			gnd::conf::get_parameter( src, &dest->error_covariance_initial_pose );
			// particle filter parameter
			gnd::conf::get_parameter( src, &dest->number_of_particles );
			gnd::conf::get_parameter( src, &dest->period_resampling );
			// debug option
			gnd::conf::get_parameter( src, &dest->period_cui_status_display );
			gnd::conf::get_parameter( src, &dest->particles_log );

			return 0;
		}



		/*
		 * @brief config file write
		 * @param [in] fname : file name
		 * @param [in] src   : configuration parameter
		 */
		inline
		int fwrite_node_config( const char* fname, node_config *src ) {
			gnd_assert(!fname, -1, "invalid null pointer argument\n" );
			gnd_assert(!src, -1, "invalid null pointer argument\n" );
			{ // ---> operation
				int ret;
				gnd::conf::file_stream fs;
				// convert configuration declaration
				if( (ret = set_node_config(&fs, src)) < 0 ) return ret;

				return fs.write(fname);
			} // <--- operation
		}

		/*
		 * @brief set config description
		 * @param [out] dest : description
		 * @param [in]   src : parameter
		 */
		inline
		int set_node_config( gnd::conf::configuration *dest, node_config *src ) {
			gnd_assert(!dest, -1, "invalid null pointer argument\n" );
			gnd_assert(!src, -1, "invalid null pointer argument\n" );

			// ros communication parameter
			gnd::conf::set_parameter( dest, &src->node_name );
			gnd::conf::set_parameter( dest, &src->topic_name_pose );
			gnd::conf::set_parameter( dest, &src->topic_name_particles );
			gnd::conf::set_parameter( dest, &src->topic_name_particle_weights );
			gnd::conf::set_parameter( dest, &src->topic_name_motion );
			gnd::conf::set_parameter( dest, &src->service_name_reset_particles_nd );
			// initial pose option
			gnd::conf::set_parameter( dest, &src->initial_pose );
			gnd::conf::set_parameter( dest, &src->error_covariance_initial_pose );
			// particle filter parameter
			gnd::conf::set_parameter( dest, &src->number_of_particles );
			gnd::conf::set_parameter( dest, &src->period_resampling );
			// debug option
			gnd::conf::set_parameter( dest, &src->period_cui_status_display );
			gnd::conf::set_parameter( dest, &src->particles_log );

			return 0;
		}
	}
}
// <--- function definition

#endif // GND_PARTICLE_LOCALIZER_CONFIG_HPP_
