/**
 * @file gnd_particle_localizer/src/main.cpp
 *
 * @brief Test Program
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include "gnd/gnd-multi-platform.h"
#include "gnd/gnd-multi-math.h"
#include "gnd/gnd_particle_localizer.hpp"

#include <stdio.h>
#include <float.h>
#include "ros/ros.h"

#include "gnd_geometry2d_msgs/msg_pose2d_stamped.h"
#include "gnd_geometry2d_msgs/msg_velocity2d_with_covariance_stamped.h"
#include "gnd_particle_localizer/msg_localization_particles2d_stamped.h"
#include "gnd_particle_localizer/msg_localization_particle_weights_stamped.h"

#include "gnd/gnd_rosmsg_reader.hpp"
#include "gnd/gnd-random.hpp"
#include "gnd/gnd-util.h"


// type definition
typedef gnd_geometry2d_msgs::msg_pose2d_stamped								msg_pose_t;
typedef gnd_geometry2d_msgs::msg_velocity2d_with_covariance_stamped			msg_motion_t;
typedef gnd::rosutil::rosmsgs_reader_stamped<msg_motion_t>					msgreader_motion_t;
typedef gnd_particle_localizer::msg_localization_particles2d_stamped		msg_particles_t;
typedef gnd_particle_localizer::msg_localization_particle_weights_stamped	msg_particle_weights_t;
typedef gnd::rosutil::rosmsgs_reader_stamped<msg_particle_weights_t>		msgreader_particle_weights_t;

int main(int argc, char **argv) {
	gnd::particle_localizer::node_config			node_config;
	{ // ---> start up, read configuration file
		if( argc > 1 ) {
			if( gnd::particle_localizer::fread_node_config( argv[1], &node_config ) < 0 ){
				char fname[1024];
				fprintf(stdout, "   ... Error: fail to read config file \"%s\"\n", argv[1]);
				sprintf(fname, "%s.tmp", argv[1]);
				// file out configuration file
				if( gnd::particle_localizer::fwrite_node_config( fname, &node_config ) >= 0 ){
					fprintf(stdout, "            : output sample configuration file \"%s\"\n", fname);
				}
				return -1;
			}
			else {
				fprintf(stdout, "   ... read config file \"%s\"\n", argv[1]);
			}
		}
	} // <--- start up, read configuration file

	{ // ---> initialize rosÅ@platform
		if( node_config.node_name.value[0] ) {
			ros::init(argc, argv, node_config.node_name.value);
		}
		else {
			fprintf(stdout, "   ... Error: node name is null, you must specify the name of this node via config item \"%s\"\n", node_config.node_name.item);
			return -1;
		}
		fprintf(stdout, " node: \"%s\"\n", node_config.node_name.value);
	} // <--- initialize rosÅ@platform

	// ros nodehandle
	ros::NodeHandle					nodehandle;					// node handle
	ros::NodeHandle					nodehandle_private("~");	// private node handle

	ros::Publisher					pub_pose;					// estimated pose message publisher
	msg_pose_t						msg_pose;					// estimated pose message

	ros::Subscriber					subsc_motion;				// motion message subscriber
	msgreader_motion_t				msgreader_motion;			// motion message reader

	ros::Publisher					pub_particles;				// particles message publisher
	msg_particles_t					msg_particles;				// particles message

	ros::Subscriber					subsc_particle_weights;		// particle weight message subscriber
	msgreader_particle_weights_t	msgreader_particle_weights;	// particle weight message reader

	FILE *fp_txtlog = 0;


	{ // ---> initialization
		int phase = 0;
		ros::Time time_start;
		fprintf(stdout, "---------- initialization ----------\n");

		// ---> show initialize phase task
		if( ros::ok() ) {
			fprintf(stdout, " => initialization task\n");
			fprintf(stdout, "   %d. make estimated pose publisher\n", ++phase);
			fprintf(stdout, "   %d. make motion subscriber\n", ++phase);
			fprintf(stdout, "   %d. make particles publisher\n", ++phase);
			fprintf(stdout, "   %d. make particle weights\n", ++phase);
			fprintf(stdout, "\n");
		} // <--- show initialize phase task

		phase = 0;
		// ---> make estimated pose publisher
		if( ros::ok() ) {
			fprintf(stdout, "\n");
			fprintf(stdout, " => %d. make estimated pose publisher\n", ++phase);

			if( !node_config.topic_name_pose.value[0] ) {
				ros::shutdown();
				fprintf(stdout, "    ... error: invalid topic name\n");
			}
			else {
				fprintf(stdout, "    ... topic name is \"%s\"\n", node_config.topic_name_pose.value );
				pub_pose = nodehandle.advertise<msg_pose_t>(node_config.topic_name_pose.value, 1000 );

				msg_pose.header.stamp = time_start;
				msg_pose.header.seq = 0;
				msg_pose.header.frame_id = "";
				msg_pose.x = node_config.initial_pose.value[0];
				msg_pose.y = node_config.initial_pose.value[1];
				msg_pose.theta = node_config.initial_pose.value[2];

				fprintf(stdout, "    ... ok\n");
			}
		} // <--- make estimated pose publisher

		// ---> make motion subscriber
		if( ros::ok() ) {
			fprintf(stdout, "\n");
			fprintf(stdout, " => %d. make motion subscriber\n", ++phase);

			if( !node_config.topic_name_motion.value[0] ) {
				ros::shutdown();
				fprintf(stdout, "    ... error: invalid topic name\n");
			}
			else {
				fprintf(stdout, "    ... topic name is \"%s\"\n", node_config.topic_name_motion.value );
				// allocate buffer
				msgreader_motion.allocate(100);

				// subscribe
				subsc_motion = nodehandle.subscribe(node_config.topic_name_motion.value, 1000,
						&msgreader_motion_t::rosmsg_read,
						msgreader_motion.reader_pointer() );
				fprintf(stdout, "    ... ok\n");
			}
		} // <--- make motion subscriber

		// ---> make particles publisher
		if( ros::ok() ) {
			fprintf(stdout, "\n");
			fprintf(stdout, " => %d. make estimated pose publisher\n", ++phase);

			if( !node_config.topic_name_particles.value[0] ) {
				ros::shutdown();
				fprintf(stdout, "    ... error: invalid topic name\n");
			}
			else {
				fprintf(stdout, "    ... topic name is \"%s\"\n", node_config.topic_name_particles.value );
				pub_particles = nodehandle.advertise<msg_particles_t>(node_config.topic_name_particles.value, 1000 );

				msg_particles.header.stamp = time_start;
				msg_particles.header.seq = 0;
				msg_particles.header.frame_id = "";
				msg_particles.poses.resize( node_config.number_of_particles.value );

				{ // ---> initialize particles
					msg_particles_t::_poses_type::value_type init_particle;
					gnd::matrix::fixed<3,3> cov;
					gnd::matrix::fixed<3,3> ws;
					gnd::vector::fixed_column<3> pos;

					for (int i = 0; i < node_config.number_of_particles.value; i++ ) {
						// set co-variance
						cov[0][0] = node_config.error_covariance_initial_pose.value[0];
						cov[0][1] = cov[1][0] = node_config.error_covariance_initial_pose.value[1];
						cov[0][2] = cov[2][0] = node_config.error_covariance_initial_pose.value[2];
						cov[1][1] = node_config.error_covariance_initial_pose.value[4];
						cov[1][2] = cov[2][1] = node_config.error_covariance_initial_pose.value[5];
						cov[2][2] = node_config.error_covariance_initial_pose.value[8];
						// create random value according to co-variance
						gnd::random_gaussian_mult(&cov, 3, &ws, &pos);
						// add random value
						init_particle.x = node_config.initial_pose.value[0] + pos[0];
						init_particle.y = node_config.initial_pose.value[1] + pos[1];
						init_particle.theta = node_config.initial_pose.value[2] + pos[2];
						// set
						msg_particles.poses[i] = init_particle;
					}
				} // <--- initialize particles

				fprintf(stdout, "    ... ok\n");
			}
		} // <--- make particles publisher

		// ---> make particle weights subscriber
		if( ros::ok() ) {
			fprintf(stdout, "\n");
			fprintf(stdout, " => %d. make particle weights subscriber\n", ++phase);

			if( !node_config.topic_name_particle_weights.value[0] ) {
				ros::shutdown();
				fprintf(stdout, "    ... error: invalid topic name\n");
			}
			else {
				fprintf(stdout, "    ... topic name is \"%s\"\n", node_config.topic_name_particle_weights.value );
				// allocate buffer
				msgreader_particle_weights.allocate(100);

				// subscribe
				subsc_particle_weights = nodehandle.subscribe(node_config.topic_name_particle_weights.value, 200,
						&msgreader_particle_weights_t::rosmsg_read,
						msgreader_particle_weights.reader_pointer() );
				fprintf(stdout, "    ... ok\n");
			}
		} // <--- make particle weights subscriber

		// ---> text log file open
		if( ros::ok() && node_config.particles_log.value[0] ) {
			fprintf(stdout, "\n");
			fprintf(stdout, " => %d. create text log file\n", ++phase );
			fprintf(stdout, "    ... try to open \"%s\"\n", node_config.particles_log.value );

			if( !(fp_txtlog = fopen( node_config.particles_log.value, "w"))  ) {
				fprintf(stdout, "    ... error: fail to open \"%s\"\n", node_config.particles_log.value );
				ros::shutdown();
			}
			else {
				fprintf(fp_txtlog, "# [1. time] [2. index] [3. x] [4. y] [5. theta]\n" );
				fprintf(stdout, "    ... ok\n");
			}
		}
		// <--- text log file open

	} // <--- initialization



	{ // ---> operation
		ros::Rate loop_rate(1000);

		double time_start = 0;
		double time_current = 0;
		double time_resampling = 0;
		double time_display = 0;

		msg_motion_t msg_motion;
		msg_particle_weights_t particle_weights_integrated;
		msg_particle_weights_t msg_particle_weights;

		bool flg_resampling = false;

		int cnt_weights_out_of_synchronization = 0;

		uint32_t seq_pose_prevdisplay = msg_pose.header.seq;
		uint32_t seq_motion_prevdisplay = msg_motion.header.seq;
		int cnt_particle_weights_display = 0;
		int cnt_resampling_display = 0;
		int nline_display = 0;

		{ // ---> initialize time variables
			time_start = ros::Time::now().toSec();
			time_current = time_start;
			time_resampling = time_start;
			time_display = time_start;
		} // <--- initialize time variables

		// initialize integrated particle weights storage
		particle_weights_integrated.weights.resize( msg_particles.poses.size(), (float) 1.0 / msg_particles.poses.size() );

		// ---> main loop
		while( ros::ok() ) {
			// blocking
			loop_rate.sleep();
			ros::spinOnce();

			time_current = ros::Time::now().toSec();


			// ---> particle transition with motion model
			if( msgreader_motion.copy_new( &msg_motion, msg_motion.header.seq ) == 0 ) {
				double trans_x, trans_y, rot;
				double cosv, sinv;

				trans_x = msg_motion.vel_x * msg_motion.measuring_period;
				trans_y = msg_motion.vel_y * msg_motion.measuring_period;
				rot = msg_motion.vel_ang * msg_motion.measuring_period;

				// pose
				cosv = cos(msg_pose.theta);
				sinv = sin(msg_pose.theta);
				msg_pose.x += cosv * trans_x - sinv * trans_y;
				msg_pose.y += sinv * trans_x + cosv * trans_y;
				msg_pose.theta += rot;

				{ // ---> particles transition
					gnd::matrix::fixed<3,3> error_covariance_vel;
					error_covariance_vel[0][0] = msg_motion.covariance[0];
					error_covariance_vel[0][1] = error_covariance_vel[1][0] = msg_motion.covariance[1];
					error_covariance_vel[0][2] = error_covariance_vel[2][0] = msg_motion.covariance[2];
					error_covariance_vel[1][1] = msg_motion.covariance[4];
					error_covariance_vel[1][2] = error_covariance_vel[2][1] = msg_motion.covariance[5];
					error_covariance_vel[2][2] = msg_motion.covariance[8];

					for( unsigned int i = 0; i < msg_particles.poses.size(); i++ ) {
						gnd::matrix::fixed<3,3> cp_error_cov, ws;
						gnd::vector::fixed_column<3> error_vel;

						// apply the velocity error with random value
						gnd::matrix::copy(&cp_error_cov, &error_covariance_vel);
						gnd::random_gaussian_mult(&cp_error_cov, 3, &ws, &error_vel);
						trans_x = ( msg_motion.vel_x + error_vel[0] ) * msg_motion.measuring_period;
						trans_y = ( msg_motion.vel_y + error_vel[1] ) * msg_motion.measuring_period;
						rot = ( msg_motion.vel_ang + error_vel[2] ) * msg_motion.measuring_period;

						// pose calculation
						cosv = cos(msg_particles.poses[i].theta);
						sinv = sin(msg_particles.poses[i].theta);
						msg_particles.poses[i].x += cosv * trans_x - sinv * trans_y;
						msg_particles.poses[i].y += sinv * trans_x + cosv * trans_y;
						msg_particles.poses[i].theta += rot;

					}
				} // <--- particles transition


				// publish
				msg_pose.header.stamp = msg_motion.header.stamp;
				msg_pose.header.seq++;
				pub_pose.publish(msg_pose);

				msg_particles.header.stamp = msg_motion.header.stamp;
				msg_particles.header.seq++;
				pub_particles.publish(msg_particles);

			} // <--- particle transition with motion model


			// ---> update particle weights by measurement model
			if( msgreader_particle_weights.copy_new(&msg_particle_weights, &msg_particle_weights.header.stamp) == 0 ) {

				if( !gnd::rosutil::is_sequence_updated( particle_weights_integrated.seq_particles,  msg_particle_weights.seq_particles) ) {
					// out of synchronization
					cnt_weights_out_of_synchronization++;
				}
				else if( msg_particle_weights.weights.size() < particle_weights_integrated.weights.size() ) {
					// some particle weights are lacked
				}
				else {
					// integrate
					double sum = 0;
					for( unsigned int i = 0; i < particle_weights_integrated.weights.size(); i++ ){
						// integrate a particle weight
						particle_weights_integrated.weights[i] *= msg_particle_weights.weights[i];

						// zero weight exception
						if( particle_weights_integrated.weights[i] <= 0  ) {
							particle_weights_integrated.weights[i] = FLT_EPSILON;
						}
						// summention
						sum += particle_weights_integrated.weights[i];
					}

					// normalize (make the sum to be equal 1.0) to avoid zero weight due to rounding error
					for( unsigned int i = 0; i < particle_weights_integrated.weights.size(); i++ ){
						particle_weights_integrated.weights[i] /= sum;
					}

					// resampling is possible
					flg_resampling = true;
				}
			} // <--- update particle weights by measurement model



			// ---> resampling according to particle weights
			if( time_current > time_resampling ) {
				// ---> resampling
				if( !flg_resampling ) {
					// particles are not evaluated
				}
				else {
					unsigned int i, j;
					msg_particles_t::_poses_type copy_poses = msg_particles.poses;

					for( i = 0; i < msg_particles.poses.size(); i++ ) {
						double key = gnd::random_uniform();	// the sum of weights is equal 1.0
						double sum = 0;

						// select a particle according to weight liner probability
						for( j = 0; j < particle_weights_integrated.weights.size(); j++ ) {
							sum += particle_weights_integrated.weights[j];
							if( sum > key ) break;
						}

						// set a new particle
						msg_particles.poses[i] = copy_poses[j];
					}

					// update sequence when resampling
					particle_weights_integrated.seq_particles = msg_particles.header.seq;
					// reset weights
					particle_weights_integrated.weights.resize( msg_particles.poses.size(), (float) 1.0 / msg_particles.poses.size() );
					flg_resampling = false;
				} // <--- resampling

				// next time
				time_resampling = gnd_loop_next(time_current, time_start, node_config.cycle_resampling.value);
			} // <--- resampling according to particle weights


			// ---> display status
			if( node_config.cycle_cui_status_display.value > 0
					&& time_current > time_display ) {

				if( nline_display ) {
					::fprintf(stderr, "\x1b[%02dA", nline_display);
					nline_display = 0;
				}

				nline_display++; ::fprintf(stderr, "\x1b[K-------------------- \x1b[1m\x1b[36m%s\x1b[39m\x1b[0m --------------------\n", node_config.node_name.value);
				nline_display++; ::fprintf(stderr, "\x1b[K operating time : %6.01lf[sec]\n", time_current - time_start);
				// position (publish)
				nline_display++; ::fprintf(stderr, "\x1b[K       position :   topic name \"%s\" (publish)\n", node_config.topic_name_pose.value);
				nline_display++; ::fprintf(stderr, "\x1b[K                :        value %8.03lf[m], %8.03lf[m], %6.01lf[deg]\n",
						msg_pose.x, msg_pose.y, gnd_ang2deg( msg_pose.theta ) );
				nline_display++; ::fprintf(stderr, "\x1b[K                :      publish %.01lf [Hz]\n",
						(double) (msg_pose.header.seq - seq_pose_prevdisplay) / node_config.cycle_cui_status_display.value );
				seq_pose_prevdisplay = msg_pose.header.seq;
				// velocity (subscribe)
				nline_display++; ::fprintf(stderr, "\x1b[K         motion :   topic name \"%s\" (subscribe)\n", node_config.topic_name_motion.value);
				nline_display++; ::fprintf(stderr, "\x1b[K                :        value %5.02lf[m/s], %4.02lf[m/s], %4.01lf[deg/s]\n",
						msg_motion.vel_x, msg_motion.vel_y, gnd_ang2deg( msg_motion.vel_ang ) );
				nline_display++; ::fprintf(stderr, "\x1b[K                :  co-variance %lf %lf %lf\n", msg_motion.covariance[0], msg_motion.covariance[1],msg_motion.covariance[2] );
				nline_display++; ::fprintf(stderr, "\x1b[K                :              %lf %lf %lf\n", msg_motion.covariance[3], msg_motion.covariance[4],msg_motion.covariance[5] );
				nline_display++; ::fprintf(stderr, "\x1b[K                :              %lf %lf %lf\n", msg_motion.covariance[6], msg_motion.covariance[7],msg_motion.covariance[8] );
				nline_display++; ::fprintf(stderr, "\x1b[K                :    subscribe %.01lf [Hz]\n",
						(double) (msg_motion.header.seq - seq_motion_prevdisplay) / node_config.cycle_cui_status_display.value );
				seq_motion_prevdisplay = msg_motion.header.seq;
				// weight (subscribe)
				nline_display++; ::fprintf(stderr, "\x1b[K        weights :   topic name \"%s\" (subscribe)\n", node_config.topic_name_particle_weights.value);
				nline_display++; ::fprintf(stderr, "\x1b[K                :    subscribe %d [cnt]\n", cnt_particle_weights_display );
				// resampling
				nline_display++; ::fprintf(stderr, "\x1b[K     resampling : count %d\n", cnt_resampling_display );
				// exception
				nline_display++; ::fprintf(stderr, "\x1b[K      exception : out of sync %d\n", cnt_weights_out_of_synchronization );


				nline_display++; ::fprintf(stderr, "\x1b[K\n");

				// update
				time_display = gnd_loop_next(time_current, time_start, node_config.cycle_cui_status_display.value );

			} // <--- display status
		} // <--- main loop
	} // <--- operation



	{ // ---> finalization
		fprintf(stderr, "\n");
		fprintf(stderr, "---------- finalization ----------\n");

		fprintf(stderr, " ... fin\n");
	} // <--- finalization
	return 0;
}
