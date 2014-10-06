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

#include "gnd/gnd_particle_localizer_config.hpp"

#endif // gnd_particle_localizer_HPP
