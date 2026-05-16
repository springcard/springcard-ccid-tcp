# SpringCard CCID TCP Driver

`springcard-ccid-tcp` is a PC/SC IFD Handler for Unix-like systems using
pcsc-lite. It lets pcscd expose supported SpringCard network readers as PC/SC
readers by carrying CCID exchanges over TCP instead of USB.

This project is derived from Ludovic Rousseau's open source CCID driver. The
USB transport layer has been removed for this driver and replaced by the
network transport used by SpringCard IP readers.

## Purpose and Support Scope

SpringCard provides this open source driver to help integrators connect
SpringCard network readers to PC/SC-based applications.

SpringCard standard technical support covers SpringCard products themselves:
reader configuration, firmware, product behavior and product documentation.
Development topics around this driver, including build issues, packaging,
porting to a specific distribution, debugging local pcsc-lite installations or
adapting the source code, are outside the standard support scope.

If you need SpringCard to assist with such work, contact us to discuss a
dedicated project or integration engagement.

SpringCard does not commit to maintaining this driver indefinitely or to
implementing future evolutions on request. Contributions are welcome through
pull requests.

## Documentation

Start with:

- `README` for the driver overview, supported readers and runtime behavior.
- `INSTALL` for build, installation and configuration instructions.

## License

This project is distributed under the GNU Lesser General Public License. See
`COPYING` and `README` for details.
