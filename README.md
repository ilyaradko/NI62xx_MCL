## Qudi branch for confocal scan with M-series NI cards and MCL piezo stage ##

This branch introduces a possibility to make confocal scans with gated counting using M-series NI cards (NI 62xx). Standard implementation only allows using X-series (NI 63xx), because of limitation on the number of counters needed for gated counting. In order M-series cards to work, an external clock signal must be provided during confocal scan. It can be another NI card or anything else. In this branch, we use pixel clock provided by MadCityLabs (MCL) piezo stage as an external clock signal for gated counting.

The branch has a new hardware module class `NationalInstrumentsMSeries` that inherits most of functionality from `NationalInstrumentsXSeries` except those related to scanner clock and analog output tasks, such that control of the piezo is done outside of this hardware module.

For MCL piezo, two hardware modules added: `MCL` class implements low-level functionality unrelated to Qudi or any kind of scans, whereas `MCLScanner` class implements external scanner clock for gated counting and control of the piezo for confocal scanning, substituting hereby scanner clock and analog output tasks of the `NationalInstrumentsXSeries` class.
