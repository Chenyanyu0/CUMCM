# Offline test report utility; paths are relative to the problem3 directory.
param(
    [string[]]$InputReports = @('bounded_estimate_v11_shard_0.json', 'bounded_estimate_v11_shard_250.json',
                              'bounded_estimate_v11_shard_500.json', 'bounded_estimate_v11_shard_750.json'),
    [string]$OutputReport = 'bounded_estimate_v11_1000.json'
)

$ErrorActionPreference = 'Stop'
Push-Location (Split-Path -Parent $PSScriptRoot)
try {
    $reports = @($InputReports | ForEach-Object { Get-Content -Raw -Encoding UTF8 -LiteralPath $_ | ConvertFrom-Json })
    if (!$reports.Count) { throw 'No reports supplied' }
    $version = $reports[0].planner_version
    foreach ($report in $reports) {
        if ($report.planner_version -ne $version -or $report.case_generator -ne 'synthetic-v1' -or
            $report.cases -ne $report.case_results.Count -or
            $report.strategies.optimized.cases_passed -ne $report.cases) {
            throw 'Reports must describe successful runs of the same version and generator'
        }
    }
    $rows = @($reports | ForEach-Object { $_.case_results } | Sort-Object seed)
    if (@($rows.seed | Select-Object -Unique).Count -ne $rows.Count) { throw 'Duplicate seeds' }
    foreach ($row in $rows) {
        if ($row.sources -ne 10 + $row.seed % 7 -or !$row.reference -or
            $row.optimized.virtual_time_s -le 0 -or $row.reference.virtual_time_s -le 0) {
            throw 'Invalid seed, source count, or paired times'
        }
    }

    function Summarize($cases) {
        $count = $cases.Count
        $times = @($cases.optimized.virtual_time_s | Sort-Object)
        $oldTimes = @($cases.reference.virtual_time_s | Sort-Object)
        $old = ($oldTimes | Measure-Object -Average).Average
        $new = ($times | Measure-Object -Average).Average
        $sources = ($cases.sources | Measure-Object -Sum).Sum
        $regressions = @($cases | ForEach-Object {
            [pscustomobject]@{ seed = $_.seed; delta_s = $_.optimized.virtual_time_s - $_.reference.virtual_time_s;
                before_s = $_.reference.virtual_time_s; after_s = $_.optimized.virtual_time_s }
        } | Sort-Object delta_s -Descending)
        return [ordered]@{
            cases_passed = $count
            sources_cleared = $sources
            mean_virtual_time_s = $new
            reference_mean_virtual_time_s = $old
            mean_time_saved_s = $old - $new
            total_time_reduction_percent = 100 * ($old - $new) / $old
            p95_virtual_time_s = $times[[int][math]::Ceiling(.95 * $count) - 1]
            reference_p95_virtual_time_s = $oldTimes[[int][math]::Ceiling(.95 * $count) - 1]
            max_virtual_time_s = $times[-1]
            reference_max_virtual_time_s = $oldTimes[-1]
            weighted_average_time_s = $new * $count / $sources
            mean_distance_m = ($cases.optimized.distance_m | Measure-Object -Average).Average
            reference_mean_distance_m = ($cases.reference.distance_m | Measure-Object -Average).Average
            mean_measurements = ($cases.optimized.measurements | Measure-Object -Average).Average
            reference_mean_measurements = ($cases.reference.measurements | Measure-Object -Average).Average
            mean_failed_clears = ($cases.optimized.failed_clears | Measure-Object -Average).Average
            probe_attempts = ($cases.planner.probe_attempts | Measure-Object -Sum).Sum
            probe_successes = ($cases.planner.probe_successes | Measure-Object -Sum).Sum
            probe_failures = ($cases.planner.probe_failures | Measure-Object -Sum).Sum
            certified_clear_failures = ($cases.planner.certified_clear_failures | Measure-Object -Sum).Sum
            one_read_failures = ($cases.planner.one_read_failures | Measure-Object -Sum).Sum
            unified_guaranteed_misses = ($cases.planner.unified_guaranteed_misses | Measure-Object -Sum).Sum
            improved_cases = @($regressions | Where-Object { $_.delta_s -lt -1e-5 }).Count
            worsened_cases = @($regressions | Where-Object { $_.delta_s -gt 1e-5 }).Count
            unchanged_cases = @($regressions | Where-Object { [math]::Abs($_.delta_s) -le 1e-5 }).Count
            largest_regression = $regressions[0]
        }
    }

    $byMode = [ordered]@{}
    foreach ($mode in 0..3) {
        $subset = @($rows | Where-Object { $_.seed % 4 -eq $mode })
        if ($subset.Count) { $byMode[[string]$mode] = Summarize $subset }
    }
    $splits = [ordered]@{}
    foreach ($name in @('pilot_0_99', 'validation_100_plus')) {
        $subset = @($rows | Where-Object { if ($name -eq 'pilot_0_99') { $_.seed -lt 100 } else { $_.seed -ge 100 } })
        if ($subset.Count) { $splits[$name] = Summarize $subset }
    }
    $merged = [ordered]@{
        cases = $rows.Count
        seed_start = $rows[0].seed
        planner_version = $version
        case_generator = 'synthetic-v1'
        description = 'Merged paired offline Mock runs; not official simulator scores'
        input_reports = $InputReports
        reference_report = $reports[0].reference_report
        reference_version = $reports[0].reference_version
        area_model = $reports[0].area_model
        window_model = $reports[0].window_model
        one_read_model = $reports[0].one_read_model
        probe_clear_model = $reports[0].probe_clear_model
        estimate_model = $reports[0].estimate_model
        clear_region_model = $reports[0].clear_region_model
        max_shard_test_time_s = ($reports.real_test_time_s | Measure-Object -Maximum).Maximum
        strategies = @{ optimized = Summarize $rows }
        error_modes = @{ '0' = 'position-dependent deterministic sine'; '1' = 'fixed +1 degree';
                         '2' = 'fixed -1 degree'; '3' = 'zero error' }
        by_error_mode = $byMode
        splits = $splits
        case_results = $rows
    }
    $merged | ConvertTo-Json -Depth 50 | Set-Content -Encoding UTF8 -LiteralPath $OutputReport
    $merged.strategies.optimized | ConvertTo-Json -Depth 4
    Write-Host "Wrote $OutputReport"
} finally {
    Pop-Location
}
