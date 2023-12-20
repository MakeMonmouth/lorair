# LoRAIR - A LoRaWAN Air Quality Monitor for Citizen Science projects

We've stolen a lot of the ideas behind the incredible [sensor.community](https://sensor.community/en/) design, but we're using LoRaWAN instead of WiFi and sending the data to Grafana.

## Contributing to the project

We'd love as many people to get involved with the project as possible, whether it's code, design, or documentation.  Here's how to get started!

### Forks, Pulls, and Issues

We understand that many people find the workflow around git complex, so we try to keep it as simple as possible.

We recommend that you use a bit of software known as an "IDE" (don't worry about what it stands for, it's what it does that is the important bit!) that includes git support, such as [VSCode](https://code.visualstudio.com/).  VSCode has a plugin for [PlatformIO](https://platformio.org/platformio-ide) (more on what that does later) and something called `git` which is what we use to control how people contribute.

Thankfully, the folks over in the VSCode team have put together a great tutorial on [how to get started with clones, forks, and pull requests](https://code.visualstudio.com/docs/sourcecontrol/github) and you'll want to make yourself familiar with that before you get involved.

If you have any questions, feel free to pop down to a [Make Monmouth](https://www.makemonmouth.co.uk/) meeting and one of our team will be happy to help!

### Documentation

Documentation is vitally important to any project, especially one that is going to be used by people who may not be experienced in technical areas.  We need our documentation to be clear, concise, and easy to follow, and we hope you'll help us achieve that!

Our documentation is written in [Markdown](https://www.markdownguide.org/) and contained within the `docs` folder.

We use Markdown for our documentation because we can easily export it to all kinds of different formats depending on who needs to read it.

If you've not written documents in Markdown before, then there is an [excellent getting started guide](https://www.markdownguide.org/getting-started/) you can follow!

### Development Environment

We use [Python Poetry](https://python-poetry.org/) for dependency management, [PlatformIO](https://platformio.org/) for the sensor code, and [mkdocs](https://www.mkdocs.org/) for the documentation.

