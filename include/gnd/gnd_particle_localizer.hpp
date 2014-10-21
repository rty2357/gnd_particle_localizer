/**
 * @file /include/gnd_particle_localizer/gnd_particle_localizer.hpp
 *
 * @brief Template class for gnd_particle_localizer.
 **/
#ifndef gnd_particle_localizer_HPP
#define gnd_particle_localizer_HPP

#ifdef WIN32
#ifdef gnd_particle_localizer_EXPORTS
#define gnd_particle_localizer_API __declspec(dllexport)
#else
#define gnd_particle_localizer_API __declspec(dllimport)
#endif
#else
#define gnd_particle_localizer_API
#endif

#include "gnd_geometry2d_msgs/msg_pose2d_stamped.h"
#include "gnd_geometry2d_msgs/msg_velocity2d_with_covariance_stamped.h"
#include "gnd_particle_localizer/msg_localization_particles2d_stamped.h"
#include "gnd_particle_localizer/msg_localization_particle_weights_stamped.h"
#include "gnd_particle_localizer/srv_reset_particles_normal_distribution.h"

#include "gnd/gnd_rosmsg_reader.hpp"

#include <float.h>

// ---> type declaration ( callback function object )
namespace gnd {
	namespace particle_localizer {
		class srv_funcobj_reset_particles_normal_distribution;
	}
} // <--- type declaration ( callback function object )


// ---> message type definition
namespace gnd {
	namespace particle_localizer {
		typedef gnd_geometry2d_msgs::msg_pose2d_stamped									msg_pose_t;
		typedef gnd_geometry2d_msgs::msg_velocity2d_with_covariance_stamped				msg_motion_t;
		typedef gnd::rosutil::rosmsgs_reader_stamped<msg_motion_t>						msgreader_motion_t;
		typedef gnd_particle_localizer::msg_localization_particles2d_stamped			msg_particles_t;
		typedef msg_particles_t::_poses_type											particles_t;
		typedef particles_t::value_type													particle_t;
		typedef gnd_particle_localizer::msg_localization_particle_weights_stamped		msg_particle_weights_t;
		typedef gnd::rosutil::rosmsgs_reader_stamped<msg_particle_weights_t>			msgreader_particle_weights_t;
		typedef msg_particle_weights_t::_weights_type									particle_weights_t;
		typedef particle_weights_t::value_type											particle_weight_t;

		typedef gnd_particle_localizer::srv_reset_particles_normal_distributionRequest	srv_reset_particles_nd_request_t;
		typedef gnd_particle_localizer::srv_reset_particles_normal_distributionResponse srv_reset_particles_nd_response_t;

		// ---> callback function object
		class srv_funcobj_reset_particles_normal_distribution {
		public:
			typedef srv_reset_particles_nd_request_t request_t;
			typedef srv_reset_particles_nd_response_t response_t;

		private:
			struct {
				double x;
				double y;
				double theta;
			} _average;
			double _covariance[9];
			int _size;
			bool _is_called;
		public:
			srv_funcobj_reset_particles_normal_distribution();
			int clear();
		public:
			bool is_called();
		public:
			int get_average(double *x, double *y, double *theta);
			int get_covariance( double *cov );
			int get_size(int *size);
		public:
			bool callback(request_t& request, response_t& response);
		};
		// <--- callback function object

	}
} // <--- message type definition


// ---> constant definition
namespace gnd {
	namespace particle_localizer {
		const particle_weight_t WEIGHT_T_EPSILON	= FLT_EPSILON;
	}
} // ---> constant definition


// ---> callback function definition
namespace gnd {
	namespace particle_localizer {
		srv_funcobj_reset_particles_normal_distribution::srv_funcobj_reset_particles_normal_distribution() {
			clear();
		}

		int srv_funcobj_reset_particles_normal_distribution::clear() {
			_is_called = false;
			return 0;
		}

		bool srv_funcobj_reset_particles_normal_distribution::is_called() {
			return _is_called;
		}


		int srv_funcobj_reset_particles_normal_distribution::get_average(double *x, double *y, double *theta) {
			*x = _average.x;
			*y = _average.y;
			*theta = _average.theta;
			return 0;
		}

		int srv_funcobj_reset_particles_normal_distribution::get_covariance( double *cov ) {
			memcpy( cov, _covariance, sizeof(_covariance) );
			return 0;
		}

		int srv_funcobj_reset_particles_normal_distribution::get_size(int *size) {
			*size = _size;
			return 0;
		}

		bool srv_funcobj_reset_particles_normal_distribution::callback(request_t& request, response_t& response) {
			if(_is_called) {
				response.ret = false;
			}
			else if( request.size <= 0 ) {
				response.ret = false;
			}
			else {
				_average.x = request.x;
				_average.y = request.y;
				_average.theta = request.theta;
				for( int i = 0; i < 9; i++){
					_covariance[0] = request.covariance[0];
				}
				_size = request.size;
				_is_called = true;
				response.ret = true;
			}

			return true;
		}

	}
} // ---> callback function definition

#include "gnd/gnd_particle_localizer_config.hpp"



#endif // gnd_particle_localizer_HPP
