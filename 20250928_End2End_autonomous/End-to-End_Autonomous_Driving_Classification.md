# End-to-End Autonomous Driving Classification

Based on the current development of autonomous driving technology (as of 2025), End-to-End (E2E) architecture has become the mainstream trend, aiming to directly map sensor inputs to vehicle control commands through a unified model or system. However, the industry has different understandings and technical approaches to "end-to-end" implementation, which can be mainly categorized into the following three types:

## 1. Modular End-to-End Architecture

**Technical Characteristics:**
This architecture preserves the traditional modular approach (such as perception, prediction, planning) in design but constructs these modules as differentiable neural network sub-modules and performs joint training and optimization at the entire system level. Its core advantage lies in **balancing performance with interpretability**.
*   **Lossless Information Transfer**: Replaces traditional manually-defined interfaces with native neural network data representations (such as BEV occupancy grids), reducing information loss and error propagation between modules.
*   **Global Optimization**: The entire system is optimized towards the final driving task (such as safely reaching the destination) rather than optimizing each sub-task in isolation.
*   **Interpretability**: Although the overall system is trained end-to-end, it can still output intermediate results (such as obstacle occupancy probabilities in bird's eye view), facilitating debugging and validation.

**Key Technologies and Typical Systems:**
*   **Core Technologies**:
    *   **BEV + Transformer**: Converts multi-camera images to Bird's Eye View and utilizes Transformer's attention mechanism to capture road topology and long-range dependencies between traffic participants.
    *   **Occupancy Networks**: Instead of relying on traditional bounding box detection, it determines whether each voxel in 3D space is occupied, enabling more precise identification of irregular obstacles (such as fallen tires, plastic bags).
*   **Typical Systems**:
    *   **Huawei ADS 3.0**: Adopts a "perception-cognition-modeling" approach, using GOD (General Obstacle Detection) network for perception and PDP (Prediction-Decision-Planning) network for prediction and planning, achieving end-to-end optimization through serial connection. The success rate in complex urban intersection scenarios reaches 98%.
    *   **Tesla FSD V12**: Based on a pure vision solution, uses multi-camera data to generate occupancy grid maps in BEV space and performs path planning and control based on this, significantly reducing dependence on high-definition maps.
    *   **Shanghai AI Laboratory UniAD**: As a representative from academia, UniAD integrates multiple modules such as perception, prediction, and planning into a unified framework, achieving global optimization through cross-module gradient propagation.

## 2. Dual-System End-to-End Architecture (Vision-Language-Action, VLA)

**Technical Characteristics:**
This is a hybrid architecture that combines the advantages of end-to-end and Large Language Models (LLM), commonly known as **VLA (Vision-Language-Action) architecture**. It includes two collaborating systems:
*   **Fast System**: An end-to-end neural network responsible for real-time processing of sensor data and generating preliminary driving trajectories or control commands, ensuring response speed.
*   **Slow System**: A large-scale Vision Language Model (VLM) responsible for high-level semantic understanding and reasoning of the current scene (such as understanding traffic sign meanings, analyzing pedestrian intentions, interpreting construction zone signs), providing decision support and correction suggestions for the fast system.

**Advantages**: Combines the efficient execution capability of end-to-end with the powerful semantic understanding and logical reasoning ability of large language models, enabling more human-like common-sense decisions in complex and ambiguous scenarios.

**Typical Systems:**
*   **Li Auto**: Explicitly proposed a dual-system end-to-end solution, where one neural network handles real-time control while another vision-language model handles scene semantic parsing and decision assistance.
*   **Changan Auto "Tianshu" Large Model**: Adopts a "brain-cerebellum" structure, where the "cerebellum" handles specific planning and control, while the "brain" is a large model-based slow system for complex reasoning.
*   **XPeng XNGP**: Also considered to adopt a VLA solution, converting road conditions into "semantic information" and then combining visual and semantic information to output decisions, improving system comprehensibility and safety.

## 3. One Model End-to-End Architecture

**Technical Characteristics:**
This is the purest and most radical end-to-end approach, aiming to **complete all autonomous driving tasks (perception, prediction, planning, control) with a unified multimodal large model**. This model is regarded as a "general foundation model for the driving domain".
*   **Unified Modeling**: Treats the driving problem analogously to language generation tasks, where the model directly generates control commands from sensor inputs and can even receive instructions through natural language interaction.
*   **Enormous Potential**: Theoretically capable of maximizing cross-task synergistic effects and achieving true autonomous learning and generalization.

**Challenges**: The model scale is extremely large, requiring enormous computational power and massive high-quality data, with high training difficulty. The decision-making process is highly "black box", and safety controllability and real-time performance remain significant challenges. Currently, there are no mature on-vehicle deployment solutions.

**Typical Systems/Projects:**
*   **DriveMM**: A novel large-scale multimodal model capable of processing images and multi-view videos, performing full tasks including perception, prediction, and planning.
*   **Wayve's GAIA-1 and LINGO-2**: Dedicated to building a single neural network model that directly maps from camera inputs to vehicle control outputs, serving as pioneer explorers of this approach.
*   **DriveGPT4**: Attempts to map driving tasks to text problems, utilizing large language models to directly generate trajectories or control commands.

## Summary and Comparison

| Approach | Core Concept | Advantages | Challenges | Typical Representatives |
| :--- | :--- | :--- | :--- | :--- |
| **Modular End-to-End** | Preserve modular structure, joint optimization | Strong interpretability, easy iteration, significant performance improvement | Still more complex compared to single model | Huawei ADS 3.0, Tesla FSD V12, UniAD |
| **Dual-System End-to-End (VLA)** | Fast execution + slow reasoning | Smarter and safer decisions, combines advantages of both | High computational requirements, complex dual-system coordination | Li Auto, Changan "Tianshu", XPeng XNGP |
| **One Model End-to-End** | One model handles everything | Simplest architecture, highest theoretical potential | Difficult training, extremely high computational requirements, serious black box issues | Wayve (GAIA-1, LINGO-2), DriveMM |

In summary, the current industry is evolving from traditional modular architectures towards end-to-end approaches. **Modular End-to-End** and **Dual-System End-to-End (VLA)** have become the preferred choices for mainstream automotive companies (such as Huawei, Tesla, Li Auto, XPeng) due to their balance of performance, safety, and engineering feasibility. **One Model End-to-End** represents the future direction of technology and is still in the exploration and breakthrough stage.