# DiagnosticNodelet (C++)

## Description

This class is a subclass of [`ConnectionBasedNodelet`](connection_based_nodelet.html).
Instances of this class can publish diagnostics messages.
This is an abstruct class.

## Note

Each subclass of this class is required to call `vital_checker_->poke()` on all callback functions so that this class can check the health of the functionality of the nodelet.

## Publishing Topic

- `~diagnostics` (`diagnostic_msgs::DiagnosticArray`):

    Diagnostic messages

## Parameter
- `~vital_rate` (Double, default: `0.1`):

    Rate to determine if the nodelet is in health.
    If the rate that the callback functions is below this parameter, error messages are displayed on diagnostics.

- `/diagnostic_nodelet/use_warn` or `~use_warn` (Bool, default: `False`):

    If this parameter is enabled, diagnostic messages on failure is displayed on `WARN` level instead of `ERROR` level.
    `/diagnostic_nodelet/use_warn` affects every nodelets that inherits this class, but it still can be overriden for each nodelet by setting `~use_warn` parameter.
