# SpringCard CCID TCP Driver (archived)

> **This repository is archived and remains available for historical reference.**

`springcard-ccid-tcp` is a PC/SC IFD Handler for Unix-like systems using pcsc-lite. It allows pcscd to expose compatible SpringCard network readers as PC/SC readers by carrying CCID exchanges over TCP rather than USB.

The driver is derived from Ludovic Rousseau's open source CCID driver, with the USB transport replaced by SpringCard's network transport.

## A transition to modern security requirements

SpringCard's early network-connected products, including FunkyGate IP, were designed in the early 2010s around lightweight communication protocols that made integration straightforward and efficient.

Since then, cybersecurity expectations have evolved substantially. Secure communication, protection against unauthorised access, and security throughout the product lifecycle are now central requirements for connected devices. The [European Cyber Resilience Act (CRA)](https://digital-strategy.ec.europa.eu/en/policies/cyber-resilience-act), whose main obligations apply from 11 December 2027, reinforces this direction.

These developments call for architectures built around protocols with security designed in from the outset. SpringCard is therefore retiring this legacy protocol for network integration.

## Repository status

This driver is no longer actively maintained and should not be selected for new projects. Its source code and documentation remain available as a reference for existing integrations.

For a new deployment or a migration from legacy equipment, [contact SpringCard](https://www.springcard.com/en/contact) to discuss an appropriate solution for your application.

## Legacy documentation

- [README](README): driver overview, supported readers and runtime behaviour.
- [INSTALL](INSTALL): build, installation and configuration instructions.

## License

This project is distributed under the GNU Lesser General Public License. See [COPYING](COPYING) and [README](README) for details.
