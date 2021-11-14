//#line 2 "/opt/ros/kinetic/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template"
// *********************************************************
//
// File autogenerated for the image_proc package
// by the dynamic_reconfigure package.
// Please do not edit.
//
// ********************************************************/

#ifndef __image_proc__RESIZECONFIG_H__
#define __image_proc__RESIZECONFIG_H__

#if __cplusplus >= 201103L
#define DYNAMIC_RECONFIGURE_FINAL final
#else
#define DYNAMIC_RECONFIGURE_FINAL
#endif

#include <dynamic_reconfigure/config_tools.h>
#include <limits>
#include <ros/node_handle.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/ParamDescription.h>
#include <dynamic_reconfigure/Group.h>
#include <dynamic_reconfigure/config_init_mutex.h>
#include <boost/any.hpp>

namespace image_proc
{
  class ResizeConfigStatics;

  class ResizeConfig
  {
  public:
    class AbstractParamDescription : public dynamic_reconfigure::ParamDescription
    {
    public:
      AbstractParamDescription(std::string n, std::string t, uint32_t l,
          std::string d, std::string e)
      {
        name = n;
        type = t;
        level = l;
        description = d;
        edit_method = e;
      }

      virtual void clamp(ResizeConfig &config, const ResizeConfig &max, const ResizeConfig &min) const = 0;
      virtual void calcLevel(uint32_t &level, const ResizeConfig &config1, const ResizeConfig &config2) const = 0;
      virtual void fromServer(const ros::NodeHandle &nh, ResizeConfig &config) const = 0;
      virtual void toServer(const ros::NodeHandle &nh, const ResizeConfig &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, ResizeConfig &config) const = 0;
      virtual void toMessage(dynamic_reconfigure::Config &msg, const ResizeConfig &config) const = 0;
      virtual void getValue(const ResizeConfig &config, boost::any &val) const = 0;
    };

    typedef boost::shared_ptr<AbstractParamDescription> AbstractParamDescriptionPtr;
    typedef boost::shared_ptr<const AbstractParamDescription> AbstractParamDescriptionConstPtr;

    // Final keyword added to class because it has virtual methods and inherits
    // from a class with a non-virtual destructor.
    template <class T>
    class ParamDescription DYNAMIC_RECONFIGURE_FINAL : public AbstractParamDescription
    {
    public:
      ParamDescription(std::string a_name, std::string a_type, uint32_t a_level,
          std::string a_description, std::string a_edit_method, T ResizeConfig::* a_f) :
        AbstractParamDescription(a_name, a_type, a_level, a_description, a_edit_method),
        field(a_f)
      {}

      T (ResizeConfig::* field);

      virtual void clamp(ResizeConfig &config, const ResizeConfig &max, const ResizeConfig &min) const
      {
        if (config.*field > max.*field)
          config.*field = max.*field;

        if (config.*field < min.*field)
          config.*field = min.*field;
      }

      virtual void calcLevel(uint32_t &comb_level, const ResizeConfig &config1, const ResizeConfig &config2) const
      {
        if (config1.*field != config2.*field)
          comb_level |= level;
      }

      virtual void fromServer(const ros::NodeHandle &nh, ResizeConfig &config) const
      {
        nh.getParam(name, config.*field);
      }

      virtual void toServer(const ros::NodeHandle &nh, const ResizeConfig &config) const
      {
        nh.setParam(name, config.*field);
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, ResizeConfig &config) const
      {
        return dynamic_reconfigure::ConfigTools::getParameter(msg, name, config.*field);
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const ResizeConfig &config) const
      {
        dynamic_reconfigure::ConfigTools::appendParameter(msg, name, config.*field);
      }

      virtual void getValue(const ResizeConfig &config, boost::any &val) const
      {
        val = config.*field;
      }
    };

    class AbstractGroupDescription : public dynamic_reconfigure::Group
    {
      public:
      AbstractGroupDescription(std::string n, std::string t, int p, int i, bool s)
      {
        name = n;
        type = t;
        parent = p;
        state = s;
        id = i;
      }

      std::vector<AbstractParamDescriptionConstPtr> abstract_parameters;
      bool state;

      virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, boost::any &config) const =0;
      virtual void updateParams(boost::any &cfg, ResizeConfig &top) const= 0;
      virtual void setInitialState(boost::any &cfg) const = 0;


      void convertParams()
      {
        for(std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = abstract_parameters.begin(); i != abstract_parameters.end(); ++i)
        {
          parameters.push_back(dynamic_reconfigure::ParamDescription(**i));
        }
      }
    };

    typedef boost::shared_ptr<AbstractGroupDescription> AbstractGroupDescriptionPtr;
    typedef boost::shared_ptr<const AbstractGroupDescription> AbstractGroupDescriptionConstPtr;

    // Final keyword added to class because it has virtual methods and inherits
    // from a class with a non-virtual destructor.
    template<class T, class PT>
    class GroupDescription DYNAMIC_RECONFIGURE_FINAL : public AbstractGroupDescription
    {
    public:
      GroupDescription(std::string a_name, std::string a_type, int a_parent, int a_id, bool a_s, T PT::* a_f) : AbstractGroupDescription(a_name, a_type, a_parent, a_id, a_s), field(a_f)
      {
      }

      GroupDescription(const GroupDescription<T, PT>& g): AbstractGroupDescription(g.name, g.type, g.parent, g.id, g.state), field(g.field), groups(g.groups)
      {
        parameters = g.parameters;
        abstract_parameters = g.abstract_parameters;
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, boost::any &cfg) const
      {
        PT* config = boost::any_cast<PT*>(cfg);
        if(!dynamic_reconfigure::ConfigTools::getGroupState(msg, name, (*config).*field))
          return false;

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = &((*config).*field);
          if(!(*i)->fromMessage(msg, n))
            return false;
        }

        return true;
      }

      virtual void setInitialState(boost::any &cfg) const
      {
        PT* config = boost::any_cast<PT*>(cfg);
        T* group = &((*config).*field);
        group->state = state;

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = boost::any(&((*config).*field));
          (*i)->setInitialState(n);
        }

      }

      virtual void updateParams(boost::any &cfg, ResizeConfig &top) const
      {
        PT* config = boost::any_cast<PT*>(cfg);

        T* f = &((*config).*field);
        f->setParams(top, abstract_parameters);

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = &((*config).*field);
          (*i)->updateParams(n, top);
        }
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &cfg) const
      {
        const PT config = boost::any_cast<PT>(cfg);
        dynamic_reconfigure::ConfigTools::appendGroup<T>(msg, name, id, parent, config.*field);

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          (*i)->toMessage(msg, config.*field);
        }
      }

      T (PT::* field);
      std::vector<ResizeConfig::AbstractGroupDescriptionConstPtr> groups;
    };

class DEFAULT
{
  public:
    DEFAULT()
    {
      state = true;
      name = "Default";
    }

    void setParams(ResizeConfig &config, const std::vector<AbstractParamDescriptionConstPtr> params)
    {
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator _i = params.begin(); _i != params.end(); ++_i)
      {
        boost::any val;
        (*_i)->getValue(config, val);

        if("interpolation"==(*_i)->name){interpolation = boost::any_cast<int>(val);}
        if("use_scale"==(*_i)->name){use_scale = boost::any_cast<bool>(val);}
        if("scale_height"==(*_i)->name){scale_height = boost::any_cast<double>(val);}
        if("scale_width"==(*_i)->name){scale_width = boost::any_cast<double>(val);}
        if("height"==(*_i)->name){height = boost::any_cast<int>(val);}
        if("width"==(*_i)->name){width = boost::any_cast<int>(val);}
      }
    }

    int interpolation;
bool use_scale;
double scale_height;
double scale_width;
int height;
int width;

    bool state;
    std::string name;

    
}groups;



//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      int interpolation;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      bool use_scale;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double scale_height;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double scale_width;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      int height;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      int width;
//#line 228 "/opt/ros/kinetic/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template"

    bool __fromMessage__(dynamic_reconfigure::Config &msg)
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();

      int count = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        if ((*i)->fromMessage(msg, *this))
          count++;

      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i ++)
      {
        if ((*i)->id == 0)
        {
          boost::any n = boost::any(this);
          (*i)->updateParams(n, *this);
          (*i)->fromMessage(msg, n);
        }
      }

      if (count != dynamic_reconfigure::ConfigTools::size(msg))
      {
        ROS_ERROR("ResizeConfig::__fromMessage__ called with an unexpected parameter.");
        ROS_ERROR("Booleans:");
        for (unsigned int i = 0; i < msg.bools.size(); i++)
          ROS_ERROR("  %s", msg.bools[i].name.c_str());
        ROS_ERROR("Integers:");
        for (unsigned int i = 0; i < msg.ints.size(); i++)
          ROS_ERROR("  %s", msg.ints[i].name.c_str());
        ROS_ERROR("Doubles:");
        for (unsigned int i = 0; i < msg.doubles.size(); i++)
          ROS_ERROR("  %s", msg.doubles[i].name.c_str());
        ROS_ERROR("Strings:");
        for (unsigned int i = 0; i < msg.strs.size(); i++)
          ROS_ERROR("  %s", msg.strs[i].name.c_str());
        // @todo Check that there are no duplicates. Make this error more
        // explicit.
        return false;
      }
      return true;
    }

    // This version of __toMessage__ is used during initialization of
    // statics when __getParamDescriptions__ can't be called yet.
    void __toMessage__(dynamic_reconfigure::Config &msg, const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__, const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__) const
    {
      dynamic_reconfigure::ConfigTools::clear(msg);
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->toMessage(msg, *this);

      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); ++i)
      {
        if((*i)->id == 0)
        {
          (*i)->toMessage(msg, *this);
        }
      }
    }

    void __toMessage__(dynamic_reconfigure::Config &msg) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
      __toMessage__(msg, __param_descriptions__, __group_descriptions__);
    }

    void __toServer__(const ros::NodeHandle &nh) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->toServer(nh, *this);
    }

    void __fromServer__(const ros::NodeHandle &nh)
    {
      static bool setup=false;

      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->fromServer(nh, *this);

      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i++){
        if (!setup && (*i)->id == 0) {
          setup = true;
          boost::any n = boost::any(this);
          (*i)->setInitialState(n);
        }
      }
    }

    void __clamp__()
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const ResizeConfig &__max__ = __getMax__();
      const ResizeConfig &__min__ = __getMin__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->clamp(*this, __max__, __min__);
    }

    uint32_t __level__(const ResizeConfig &config) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      uint32_t level = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->calcLevel(level, config, *this);
      return level;
    }

    static const dynamic_reconfigure::ConfigDescription &__getDescriptionMessage__();
    static const ResizeConfig &__getDefault__();
    static const ResizeConfig &__getMax__();
    static const ResizeConfig &__getMin__();
    static const std::vector<AbstractParamDescriptionConstPtr> &__getParamDescriptions__();
    static const std::vector<AbstractGroupDescriptionConstPtr> &__getGroupDescriptions__();

  private:
    static const ResizeConfigStatics *__get_statics__();
  };

  template <> // Max and min are ignored for strings.
  inline void ResizeConfig::ParamDescription<std::string>::clamp(ResizeConfig &config, const ResizeConfig &max, const ResizeConfig &min) const
  {
    (void) config;
    (void) min;
    (void) max;
    return;
  }

  class ResizeConfigStatics
  {
    friend class ResizeConfig;

    ResizeConfigStatics()
    {
ResizeConfig::GroupDescription<ResizeConfig::DEFAULT, ResizeConfig> Default("Default", "", 0, 0, true, &ResizeConfig::groups);
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.interpolation = 0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.interpolation = 4;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.interpolation = 1;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(ResizeConfig::AbstractParamDescriptionConstPtr(new ResizeConfig::ParamDescription<int>("interpolation", "int", 0, "Interpolation algorithm between source image pixels", "{'enum_description': 'interpolation type', 'enum': [{'srcline': 11, 'description': 'Nearest neighbor', 'srcfile': '/home/ur3/catkin_ws/src/lab2andDriver/drivers/camera_calibration/image_pipeline/image_proc/cfg/Resize.cfg', 'cconsttype': 'const int', 'value': 0, 'ctype': 'int', 'type': 'int', 'name': 'NN'}, {'srcline': 12, 'description': 'Linear', 'srcfile': '/home/ur3/catkin_ws/src/lab2andDriver/drivers/camera_calibration/image_pipeline/image_proc/cfg/Resize.cfg', 'cconsttype': 'const int', 'value': 1, 'ctype': 'int', 'type': 'int', 'name': 'Linear'}, {'srcline': 13, 'description': 'Cubic', 'srcfile': '/home/ur3/catkin_ws/src/lab2andDriver/drivers/camera_calibration/image_pipeline/image_proc/cfg/Resize.cfg', 'cconsttype': 'const int', 'value': 2, 'ctype': 'int', 'type': 'int', 'name': 'Cubic'}, {'srcline': 14, 'description': 'Lanczos4', 'srcfile': '/home/ur3/catkin_ws/src/lab2andDriver/drivers/camera_calibration/image_pipeline/image_proc/cfg/Resize.cfg', 'cconsttype': 'const int', 'value': 4, 'ctype': 'int', 'type': 'int', 'name': 'Lanczos4'}]}", &ResizeConfig::interpolation)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(ResizeConfig::AbstractParamDescriptionConstPtr(new ResizeConfig::ParamDescription<int>("interpolation", "int", 0, "Interpolation algorithm between source image pixels", "{'enum_description': 'interpolation type', 'enum': [{'srcline': 11, 'description': 'Nearest neighbor', 'srcfile': '/home/ur3/catkin_ws/src/lab2andDriver/drivers/camera_calibration/image_pipeline/image_proc/cfg/Resize.cfg', 'cconsttype': 'const int', 'value': 0, 'ctype': 'int', 'type': 'int', 'name': 'NN'}, {'srcline': 12, 'description': 'Linear', 'srcfile': '/home/ur3/catkin_ws/src/lab2andDriver/drivers/camera_calibration/image_pipeline/image_proc/cfg/Resize.cfg', 'cconsttype': 'const int', 'value': 1, 'ctype': 'int', 'type': 'int', 'name': 'Linear'}, {'srcline': 13, 'description': 'Cubic', 'srcfile': '/home/ur3/catkin_ws/src/lab2andDriver/drivers/camera_calibration/image_pipeline/image_proc/cfg/Resize.cfg', 'cconsttype': 'const int', 'value': 2, 'ctype': 'int', 'type': 'int', 'name': 'Cubic'}, {'srcline': 14, 'description': 'Lanczos4', 'srcfile': '/home/ur3/catkin_ws/src/lab2andDriver/drivers/camera_calibration/image_pipeline/image_proc/cfg/Resize.cfg', 'cconsttype': 'const int', 'value': 4, 'ctype': 'int', 'type': 'int', 'name': 'Lanczos4'}]}", &ResizeConfig::interpolation)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.use_scale = 0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.use_scale = 1;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.use_scale = 1;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(ResizeConfig::AbstractParamDescriptionConstPtr(new ResizeConfig::ParamDescription<bool>("use_scale", "bool", 0, "Flag to use scale instead of static size.", "", &ResizeConfig::use_scale)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(ResizeConfig::AbstractParamDescriptionConstPtr(new ResizeConfig::ParamDescription<bool>("use_scale", "bool", 0, "Flag to use scale instead of static size.", "", &ResizeConfig::use_scale)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.scale_height = 0.0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.scale_height = 10.0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.scale_height = 1.0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(ResizeConfig::AbstractParamDescriptionConstPtr(new ResizeConfig::ParamDescription<double>("scale_height", "double", 0, "Scale of height.", "", &ResizeConfig::scale_height)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(ResizeConfig::AbstractParamDescriptionConstPtr(new ResizeConfig::ParamDescription<double>("scale_height", "double", 0, "Scale of height.", "", &ResizeConfig::scale_height)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.scale_width = 0.0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.scale_width = 10.0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.scale_width = 1.0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(ResizeConfig::AbstractParamDescriptionConstPtr(new ResizeConfig::ParamDescription<double>("scale_width", "double", 0, "Scale of width", "", &ResizeConfig::scale_width)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(ResizeConfig::AbstractParamDescriptionConstPtr(new ResizeConfig::ParamDescription<double>("scale_width", "double", 0, "Scale of width", "", &ResizeConfig::scale_width)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.height = -1;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.height = 2147483647;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.height = -1;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(ResizeConfig::AbstractParamDescriptionConstPtr(new ResizeConfig::ParamDescription<int>("height", "int", 0, "Destination height. Ignored if negative.", "", &ResizeConfig::height)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(ResizeConfig::AbstractParamDescriptionConstPtr(new ResizeConfig::ParamDescription<int>("height", "int", 0, "Destination height. Ignored if negative.", "", &ResizeConfig::height)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.width = -1;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.width = 2147483647;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.width = -1;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(ResizeConfig::AbstractParamDescriptionConstPtr(new ResizeConfig::ParamDescription<int>("width", "int", 0, "Destination width. Ignored if negative.", "", &ResizeConfig::width)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(ResizeConfig::AbstractParamDescriptionConstPtr(new ResizeConfig::ParamDescription<int>("width", "int", 0, "Destination width. Ignored if negative.", "", &ResizeConfig::width)));
//#line 245 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.convertParams();
//#line 245 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __group_descriptions__.push_back(ResizeConfig::AbstractGroupDescriptionConstPtr(new ResizeConfig::GroupDescription<ResizeConfig::DEFAULT, ResizeConfig>(Default)));
//#line 366 "/opt/ros/kinetic/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template"

      for (std::vector<ResizeConfig::AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); ++i)
      {
        __description_message__.groups.push_back(**i);
      }
      __max__.__toMessage__(__description_message__.max, __param_descriptions__, __group_descriptions__);
      __min__.__toMessage__(__description_message__.min, __param_descriptions__, __group_descriptions__);
      __default__.__toMessage__(__description_message__.dflt, __param_descriptions__, __group_descriptions__);
    }
    std::vector<ResizeConfig::AbstractParamDescriptionConstPtr> __param_descriptions__;
    std::vector<ResizeConfig::AbstractGroupDescriptionConstPtr> __group_descriptions__;
    ResizeConfig __max__;
    ResizeConfig __min__;
    ResizeConfig __default__;
    dynamic_reconfigure::ConfigDescription __description_message__;

    static const ResizeConfigStatics *get_instance()
    {
      // Split this off in a separate function because I know that
      // instance will get initialized the first time get_instance is
      // called, and I am guaranteeing that get_instance gets called at
      // most once.
      static ResizeConfigStatics instance;
      return &instance;
    }
  };

  inline const dynamic_reconfigure::ConfigDescription &ResizeConfig::__getDescriptionMessage__()
  {
    return __get_statics__()->__description_message__;
  }

  inline const ResizeConfig &ResizeConfig::__getDefault__()
  {
    return __get_statics__()->__default__;
  }

  inline const ResizeConfig &ResizeConfig::__getMax__()
  {
    return __get_statics__()->__max__;
  }

  inline const ResizeConfig &ResizeConfig::__getMin__()
  {
    return __get_statics__()->__min__;
  }

  inline const std::vector<ResizeConfig::AbstractParamDescriptionConstPtr> &ResizeConfig::__getParamDescriptions__()
  {
    return __get_statics__()->__param_descriptions__;
  }

  inline const std::vector<ResizeConfig::AbstractGroupDescriptionConstPtr> &ResizeConfig::__getGroupDescriptions__()
  {
    return __get_statics__()->__group_descriptions__;
  }

  inline const ResizeConfigStatics *ResizeConfig::__get_statics__()
  {
    const static ResizeConfigStatics *statics;

    if (statics) // Common case
      return statics;

    boost::mutex::scoped_lock lock(dynamic_reconfigure::__init_mutex__);

    if (statics) // In case we lost a race.
      return statics;

    statics = ResizeConfigStatics::get_instance();

    return statics;
  }

//#line 11 "/home/ur3/catkin_ws/src/lab2andDriver/drivers/camera_calibration/image_pipeline/image_proc/cfg/Resize.cfg"
      const int Resize_NN = 0;
//#line 12 "/home/ur3/catkin_ws/src/lab2andDriver/drivers/camera_calibration/image_pipeline/image_proc/cfg/Resize.cfg"
      const int Resize_Linear = 1;
//#line 13 "/home/ur3/catkin_ws/src/lab2andDriver/drivers/camera_calibration/image_pipeline/image_proc/cfg/Resize.cfg"
      const int Resize_Cubic = 2;
//#line 14 "/home/ur3/catkin_ws/src/lab2andDriver/drivers/camera_calibration/image_pipeline/image_proc/cfg/Resize.cfg"
      const int Resize_Lanczos4 = 4;
}

#undef DYNAMIC_RECONFIGURE_FINAL

#endif // __RESIZERECONFIGURATOR_H__
