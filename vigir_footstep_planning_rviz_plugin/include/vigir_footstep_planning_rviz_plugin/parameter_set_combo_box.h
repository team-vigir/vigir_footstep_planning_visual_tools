//=================================================================================================
// Copyright (c) 2018, Stephanie Ferreira, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================


#ifndef PARAMETER_SET_COMBO_BOX_H
#define PARAMETER_SET_COMBO_BOX_H

#ifndef Q_MOC_RUN

#include <qcombobox.h>
#include <actionlib/client/simple_action_client.h>
#include <vigir_generic_params/generic_params_msgs.h>

#endif

typedef actionlib::SimpleActionClient<vigir_generic_params::GetAllParameterSetsAction > GetAllParameterSetsActionClient;

namespace vigir_footstep_planning_rviz_plugin
{
class ParameterSetComboBox : public QComboBox
{
    Q_OBJECT
public:
    ParameterSetComboBox(QWidget *parent = 0);
    ~ParameterSetComboBox();
    void initialize();

Q_SIGNALS:
    void actionClientConnected(QString name, bool connected);

public Q_SLOTS:
    void updateParameterSetNames();

private:
    GetAllParameterSetsActionClient parameter_set_ac;
    void getParameterSetsCallback(const actionlib::SimpleClientGoalState& state, const vigir_generic_params::GetAllParameterSetsResultConstPtr& result);

};

} // end namespace vigir_footstep_planning_rviz_plugin
#endif // PARMETER_SET_COMBO_BOX_H
