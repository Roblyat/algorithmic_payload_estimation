---
# Scientific Contribution

## Table of Contents
1. [What is on the market](#1-what-is-on-the-market)
2. [Questions](#2-questions)
3. [Important things we need to remember](#3-important-things-we-need-to-remember)
---

## 1) What is on the market / SIGNIFICANT PAPER / **Key References?**

## 2) Questions

- What about these observers (classical model-based observer methods)? Why can't they work without accurate model dynamics?
    - Disturbance observer
    - Dynamic state observer
    - Momentum-based observer

- How does the best mathematical expression for my problem look?
---

# Disposition

## 1) Problem Analysis

### a) Preliminary Working Title

* Summarizes the content of the thesis in brief
* What was done in the thesis?
* Which context/environment is addressed?
* Is the title specific enough?

### b) Problem Description

* Describes the intended task as precisely as possible
* Subdivision into problem statement / motivation & research question

#### 1) Problem Statement / Problem Description

* Explains the subject area & outlines the context
* Description of the relevance from:

  * a scientific perspective
  * an application-oriented perspective

#### 2) Motivation / Scientific Research Gap

* Are there comparable works?
* Do these address the intended tasks?
* Practical relevance:

  * Is the solution needed in industry/companies?
  * Demonstrated through studies, surveys, literature sources, etc.

### c) Checklist

* 3 most important facts about the context
* Why does it exist?
* Why does this thesis need to be written?

---

## 2) Research Question

* What is to be investigated?

  * **Define clearly!!!**
* Represents the substantive basis of the thesis
* Can be subdivided into sub-questions

  * Specification into individual sections
* One central question & concrete sub-questions

### Checklist

* My thesis answers the following questions …
* My thesis is relevant because …
* Why is my thesis relevant for others?
* Are there gaps in the state of the art?
* The answer is not obvious, because …

---

## 3) Methodological Approach

* Describes the scientific procedure in the thesis
* Which methods are used to address the research question(s) & sub-questions?
* Justify the selection of methods

---

## 4) Expected Results

* Rough outline of the expected results (related to research questions & sub-questions)

  * How will the result of each chapter look?

### Checklist

* My thesis shows …
* Results in relation to research question(s) & sub-questions

---

![Focus Check](/docs/reserach/illustrations/focuscheck.png)
![Structure](/docs/reserach/illustrations/structure.png)
![Validate Questions 0](/docs/reserach/illustrations/validate_questions_0.png)
![Validate Questions 1](/docs/reserach/illustrations/validate_questions_1.png)

---
---
---

# 1 Introduction

## 1.1 Motivation

### 1.1.1 Context

* robotic arm manipulators increase more and more
  - since a very long time 📍 *bottom of the funnel* → $\textcolor{orange}{funnel_0}$
* collaborative robotic arm manipulation increases more & more 📍 $\textcolor{orange}{funnel_1}$ $\textcolor{violet}{c_1}$
    - safe manipulation, safe manipulation at payloads, safe collaborative manipulation with & without payload
        - awareness of payload 📍 $\textcolor{orange}{funnel_2}$
            - 📍 awareness fundamental for safe manipulation of payloads
                - and for collaborative payload manipulation

### 1.1.2 Use Case

* camera able to get shape and dimensions of payload, but no information of mass, CoM & inertia
* payload’s mass, CoM & inertia information fundamental for safe manipulation
* payload’s mass, CoM & inertia just identifiable with sensor data
* 🔑 methods to identify robot dynamic parameters and payload parameters is relevant for robotic arm manipulation tasks like pick & place tasks or collaborative manipulation
    - 📍 robot dynamic parameter identification (RDPI) $\textcolor{violet}{c_2}$
        - relevant for general robotic arm movement
    - 📍 payload dynamic parameter identification (PDPI) $\textcolor{violet}{c_2}$
        - relevant for pick & place and collaborative manipulation

    - both dynamic online & including everything $\textcolor{violet}{c_2}$
    - to estimate online what’s going on dynamically $[vel, acc, f/t]$
      - regression respecting friction, non linearity, noise

* 🗝️ methods relevant to all robotic arm manipulation tasks $\textcolor{violet}{c_1}$
  - payload
  - surgery
  - collaborative
  - regression respecting friction, nonlinearity, noise

  ## 1.2 Problem Statement

  - to get information about the robot or payloads dynmaic parameters is not straight forward. There are problems for example non linearity and noise. 


  ## 1.3 Aim of this work
  - We want to eliminate the need for the dynamic parameters of the robot

  ### 1.3.1 Research Question

  - (using a GAN is robust at this, but lacks of anything) what needs to be done to handle that lack?

  ### 1.3.2 Scientific Contribution
  - evaluating a GAN for payload dynmaic parameter identification and impoving the lack of anything.
















---
---

what i in very far aim for is payload dynamic parameter identification, as now perfectly shown in your rewrote mathematical subsection. now i think of it an other way. i saw many approaches in current state of art mapping directly encoder/motor/sensor data to payload parameters. as well as doing the way i see it, giving the robot a awareness of its own movement, training with no payload, exactly what f/t results at the end effector. then manipulating and predicting based on $(\mathbf{a},\boldsymbol{\alpha},\boldsymbol{\omega})$ what would have result without payload based on that trajectory data. now this can be done however, mapping the motor/encoder data to f/t sensor data, being able to use the motor current as well somehow if advantages. or mapping encoder data to the motor current data, using the current in each motor and calculate joint torque by \boldsymbol{\tau}_{\mathrm{motor}} = k_t\,\boldsymbol{I} and mapping this with jacobian to EE.

now in current SoA i also see both of this but focusing on the payload parameter identification, or focusing on the robot dynamic parameter identification but then estimating robot base parameters, bases of 60 or whatever high number of predicted base parameters, to then predict joint torque or even mapping with jacobian in joint space or parameter space. 

now i see the payload parameter identification, but this is what i want to advance by setting a good bases by giving the robot a exact awareness of its own movement. now since i want to advance PDPI what i think of can be called kind of gripper or tool compensation, what already provides very good task operation as well as teleoperation. now there are i think 5 studies i know dealing with this with state of art methods like DeLaN networks + TCN (for backlash, prediction based on sequence/time window) or any Physical Informed Neural Network (PINN) or LSTM for the whole problem. I want to contribute at this point, i see a DeLaN or any possible fitting PINN plus a LSTM for backlash etc. where DeLaN PINN handles friction, Stirbeck or viscous coulomb friction model.

so now i know in this text is much method and aim of work stuff. but this is the core idea what i think of. 

now in the mathematical i want to show this, our current version aim to PDPI with the core compensation idea. now can we form that to focusing on the compensation, but seeing the PDPI.

now our current version is very good, very precise, very good scientific phrased so if we can use as much of that would be very good. and we need to still keep this as short as possible, it is okay if it gets a tiny but longer, but only if it need to be so. and also dont get any methods or aim of work content in please, i just provided that to show my core idea and perspective on that problem. 

---

okay, please lets get that more correct:
        For a gripper–payload combination we define an effective rigid body
        \begin{equation}
        \boldsymbol{\phi}_{\mathrm{eff}}
        =
        \boldsymbol{\phi}_{\mathrm{payload}}
        +
        \boldsymbol{\phi}_{\mathrm{gripper}},
        \label{eq:rigidEffective}
        \end{equation}

we are talking here about wrench, phi effective is the wrench appearing at the robots tool flange. if we would install a ft sensor at the tool flange, we would measure the effective wrench. now this wrench results in the robots motion it self, with a clean flange, increases proportional to the tool/grippers rigid body parameters phi gripper/tool and then this what remains the same every time manipulation with the gripper/tool, until manipulation payload or interaction with environment. with a clean flange we would remain with external forces, eg forces resulting interacting or colliding with the environment. so this can be observed in joint space, for each motor, however the interaction/manipulation looks like, different joint torque will appear at affected joint by the interaction/manipulation. but since we want to enhance PDPI we want to observe this in the EE frame -> measurement fame.

so we can use phi for this:
now this wrench results in the robots motion it self, with a clean flange, increases proportional to the tool/grippers rigid body parameters phi gripper/tool and then this what remains the same every time manipulation with the gripper/tool, until manipulation payload or interaction with environment.

because phi indecates rigid body that influences the effecting wrench. but i want to get that clean

---