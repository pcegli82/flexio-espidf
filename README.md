# FlexIO ESP-IDF Component

ESP-IDF driver for the NXP PCAL6416 I/O expander with open-drain style outputs.

## Usage

Add the dependency to your component:

```yaml
# main/idf_component.yml in your ESP-IDF project
dependencies:
  flexio:
    git: "https://github.com/<dein-user>/flexio-espidf.git"
    version: "1.0.0"
