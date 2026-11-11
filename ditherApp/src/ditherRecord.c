/* Record support for the "dither" record: continually moves a motor record
 * back and forth about its setpoint, within the motor's soft limits. */
#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <alarm.h>
#include <dbAccess.h>
#include <dbDefs.h>
#include <dbEvent.h>
#include <dbCommon.h>
#include <errlog.h>
#include <recGbl.h>
#include <recSup.h>

#define GEN_SIZE_OFFSET
#include "ditherRecord.h"
#undef GEN_SIZE_OFFSET
#include <epicsExport.h>

/* Motor record MSTA bits (see motor.h, msta_field) */
#define MSTA_PLUS_LIMIT_SWITCH   0x00000004u /* RA_PLUS_LS */
#define MSTA_ENCODER_SLIP_STALL  0x00000040u /* EA_SLIP_STALL */
#define MSTA_DRIVER_PROBLEM      0x00000200u /* RA_PROBLEM */
#define MSTA_CONTROLLER_COMM_ERR 0x00001000u /* CNTRL_COMM_ERR */
#define MSTA_MINUS_LIMIT_SWITCH  0x00002000u /* RA_MINUS_LS */
#define MSTA_FAULT_MASK \
    (MSTA_PLUS_LIMIT_SWITCH | MSTA_MINUS_LIMIT_SWITCH | \
     MSTA_ENCODER_SLIP_STALL | MSTA_DRIVER_PROBLEM | \
     MSTA_CONTROLLER_COMM_ERR)

/* Motor record SPMG menu (motorSPMG): Stop=0, Pause=1, Move=2, Go=3 */
#define SPMG_GO 3

enum phase { IDLE, DITHER, STOPPING, RETURNING, FAULT };

typedef struct {
    enum phase phase;
    double center, plus, minus, old_velocity, target;
    int direction, owns_velocity, stop_sent, enabled;
} Dither;

/* Snapshot of the motor fields the controller needs. */
typedef struct {
    double val, rbv, velo, vbas, vmax, llm, hlm, mres;
    epicsInt32 dmov, movn, set, spmg, lvio, miss, limits;
    epicsUInt32 msta;
} Motor;

#define report NULL
#define initialize NULL
static long init_record(struct dbCommon *, int);
static long process(struct dbCommon *);
static long special(dbAddr *, int);
#define get_value NULL
#define cvt_dbaddr NULL
#define get_array_info NULL
#define put_array_info NULL
#define get_units NULL
static long get_precision(const struct dbAddr *, long *);
#define get_enum_str NULL
#define get_enum_strs NULL
#define put_enum_str NULL
#define get_graphic_double NULL
static long get_control_double(const struct dbAddr *, struct dbr_ctrlDouble *);
#define get_alarm_double NULL

struct typed_rset ditherRSET = {
    RSETNUMBER, report, initialize, init_record, process, special, get_value,
    cvt_dbaddr, get_array_info, put_array_info, get_units, get_precision,
    get_enum_str, get_enum_strs, put_enum_str, get_graphic_double,
    get_control_double, get_alarm_double
};
epicsExportAddress(rset, ditherRSET);

static long get_precision(const struct dbAddr *paddr, long *precision)
{
    ditherRecord *prec = (ditherRecord *)paddr->precord;
    short motor_prec;
    *precision = prec->prec;
    /* follow the motor record's PREC */
    if (prec->mpre.type == DB_LINK &&
        !dbGetLink(&prec->mpre, DBR_SHORT, &motor_prec, 0, 0)) {
        prec->prec = motor_prec;
        *precision = motor_prec;
    }
    recGblGetPrec(paddr, precision);
    return 0;
}

/* PLUS and MINUS may not be negative: clamp whatever was written to 0 */
static long special(dbAddr *paddr, int after)
{
    ditherRecord *prec = (ditherRecord *)paddr->precord;
    int fieldIndex = dbGetFieldIndex(paddr);
    if (!after) return 0;
    if (fieldIndex == ditherRecordABS) {
        double half = prec->abs >= 0 ? prec->abs / 2 : 0;
        prec->abs = 2 * half;
        prec->plus = half;
        prec->minus = half;
        db_post_events(prec, &prec->abs, DBE_VALUE | DBE_LOG);
        db_post_events(prec, &prec->plus, DBE_VALUE | DBE_LOG);
        db_post_events(prec, &prec->minus, DBE_VALUE | DBE_LOG);
    } else if (fieldIndex == ditherRecordPLUS || fieldIndex == ditherRecordMINUS) {
        if (!(prec->plus >= 0)) prec->plus = 0;
        if (!(prec->minus >= 0)) prec->minus = 0;
        prec->abs = prec->plus + prec->minus;
        db_post_events(prec, &prec->plus, DBE_VALUE | DBE_LOG);
        db_post_events(prec, &prec->minus, DBE_VALUE | DBE_LOG);
        db_post_events(prec, &prec->abs, DBE_VALUE | DBE_LOG);
    }
    return 0;
}

static long get_control_double(const struct dbAddr *paddr,
                               struct dbr_ctrlDouble *pcd)
{
    int fieldIndex = dbGetFieldIndex(paddr);
    if (fieldIndex == ditherRecordPLUS || fieldIndex == ditherRecordMINUS ||
        fieldIndex == ditherRecordABS) {
        /* 0..0 would read as "no limits" to clients, so give a real upper bound */
        pcd->lower_ctrl_limit = 0;
        pcd->upper_ctrl_limit = DBL_MAX;
    } else
        recGblGetControlDouble(paddr, pcd);
    return 0;
}

static long init_record(struct dbCommon *pcommon, int pass)
{
    ditherRecord *prec = (ditherRecord *)pcommon;
    if (pass == 0) return 0;
    prec->dpvt = calloc(1, sizeof(Dither));
    if (!prec->dpvt) {
        errlogPrintf("%s: cannot allocate dither state\n", prec->name);
        return -1;
    }
    prec->udf = FALSE;
    prec->abs = prec->plus + prec->minus;
    return 0;
}

static void fail(ditherRecord *prec, Dither *d, const char *message)
{
    epicsInt32 stop = 1;
    if (d->phase != FAULT) {
        errlogPrintf("%s: %s\n", prec->name, message);
        strncpy(prec->err, message, sizeof(prec->err) - 1);
        prec->err[sizeof(prec->err) - 1] = '\0';
        if (d->owns_velocity && prec->mstp.type == DB_LINK &&
            dbPutLink(&prec->mstp, DBR_LONG, &stop, 1))
            errlogPrintf("%s: unable to stop motor\n", prec->name);
    }
    d->phase = FAULT;
}

static int put_double(ditherRecord *prec, DBLINK *link, double value)
{
    long status = dbPutLink(link, DBR_DOUBLE, &value, 1);
    if (status)
        errlogPrintf("%s: motor write failed (%ld)\n", prec->name, status);
    return status == 0;
}

static int all_links_local(ditherRecord *prec)
{
    DBLINK *links[] = { &prec->mstp, &prec->mrbv, &prec->mval, &prec->mvel, &prec->mvbs,
        &prec->mvmx, &prec->mdmv, &prec->mmvn, &prec->mllm, &prec->mhlm,
        &prec->mdll, &prec->mdhl, &prec->mmst, &prec->mset, &prec->mspm,
        &prec->mlvi, &prec->mmis, &prec->mres, &prec->mpre };
    size_t i;
    for (i = 0; i < sizeof(links) / sizeof(links[0]); ++i)
        if (links[i]->type != DB_LINK) return 0;
    return 1;
}

static int read_motor(ditherRecord *prec, Motor *m)
{
    double dllm, dhlm;
    long bad = 0;
    bad |= dbGetLink(&prec->mval, DBR_DOUBLE, &m->val, 0, 0);
    bad |= dbGetLink(&prec->mrbv, DBR_DOUBLE, &m->rbv, 0, 0);
    bad |= dbGetLink(&prec->mvel, DBR_DOUBLE, &m->velo, 0, 0);
    bad |= dbGetLink(&prec->mvbs, DBR_DOUBLE, &m->vbas, 0, 0);
    bad |= dbGetLink(&prec->mvmx, DBR_DOUBLE, &m->vmax, 0, 0);
    bad |= dbGetLink(&prec->mllm, DBR_DOUBLE, &m->llm, 0, 0);
    bad |= dbGetLink(&prec->mhlm, DBR_DOUBLE, &m->hlm, 0, 0);
    bad |= dbGetLink(&prec->mdll, DBR_DOUBLE, &dllm, 0, 0);
    bad |= dbGetLink(&prec->mdhl, DBR_DOUBLE, &dhlm, 0, 0);
    bad |= dbGetLink(&prec->mres, DBR_DOUBLE, &m->mres, 0, 0);
    bad |= dbGetLink(&prec->mdmv, DBR_LONG, &m->dmov, 0, 0);
    bad |= dbGetLink(&prec->mmvn, DBR_LONG, &m->movn, 0, 0);
    bad |= dbGetLink(&prec->mset, DBR_LONG, &m->set, 0, 0);
    bad |= dbGetLink(&prec->mspm, DBR_LONG, &m->spmg, 0, 0);
    bad |= dbGetLink(&prec->mlvi, DBR_LONG, &m->lvio, 0, 0);
    bad |= dbGetLink(&prec->mmis, DBR_LONG, &m->miss, 0, 0);
    bad |= dbGetLink(&prec->mmst, DBR_ULONG, &m->msta, 0, 0);
    /* Soft limits are disabled in the motor record when both DLLM and DHLM are 0 */
    m->limits = dllm != 0.0 || dhlm != 0.0;
    m->mres = fabs(m->mres);
    return bad == 0;
}

static int limits_invalid(const Motor *m)
{
    return !isfinite(m->llm) || !isfinite(m->hlm) || m->llm >= m->hlm;
}

static int motor_faulted(const Motor *m)
{
    return (m->msta & MSTA_FAULT_MASK) || m->set || m->spmg != SPMG_GO ||
           m->lvio;
}

static void run(ditherRecord *prec, Dither *d, const Motor *m)
{
    int enabled = d->enabled;
    int done = m->dmov, moving = m->movn;
    double positive = prec->plus, negative = prec->minus;
    double velocity = prec->velo, lo, hi;
    epicsInt32 stop = 1;
    int was_fault = d->phase == FAULT;

    if (d->phase != IDLE && d->phase != FAULT &&
        (motor_faulted(m) || (done && m->miss)))
        fail(prec, d, "Motor fault, limit, or not in Go");
    if ((d->phase == DITHER || d->phase == RETURNING) && m->limits &&
        (limits_invalid(m) || d->center < m->llm || d->center > m->hlm ||
         (!done && d->phase == DITHER &&
          (d->target < m->llm || d->target > m->hlm))))
        fail(prec, d, "Soft limits changed during move");

    if (d->phase == IDLE && enabled) {
        if (!done || moving || motor_faulted(m) || m->miss) {
            fail(prec, d, "Motor not ready");
        } else if (!isfinite(m->val) || !isfinite(positive) || positive < 0 ||
                   !isfinite(negative) || negative < 0 ||
                   positive + negative <= 0 ||
                   !isfinite(velocity) || velocity <= 0 ||
                   !isfinite(m->vbas) || !isfinite(m->vmax) ||
                   !isfinite(m->velo) || !isfinite(m->mres) || m->mres == 0 ||
                   velocity < m->vbas || (m->vmax > 0 && velocity > m->vmax)) {
            fail(prec, d, "Invalid distances or velocity");
        } else if (m->limits && (limits_invalid(m) || m->val < m->llm ||
                                 m->val > m->hlm)) {
            fail(prec, d, "Invalid limits or center");
        } else {
            d->center = m->val;
            d->target = m->val;
            d->plus = positive;
            d->minus = negative;
            d->old_velocity = m->velo;
            d->direction = 1;
            d->owns_velocity = 1;
            d->phase = DITHER;
            if (!put_double(prec, &prec->mvel, velocity))
                fail(prec, d, "Cannot set dither velocity");
            else
                prec->err[0] = '\0';
        }
    }

    if (d->phase == DITHER && !enabled) {
        if (dbPutLink(&prec->mstp, DBR_LONG, &stop, 1))
            fail(prec, d, "Cannot stop motor");
        else {
            d->phase = STOPPING;
            d->stop_sent = 1;
        }
    } else if (d->phase == DITHER) {
        if (m->limits && (limits_invalid(m) || d->center < m->llm ||
                          d->center > m->hlm ||
                          ((d->target < m->llm || d->target > m->hlm) && !done))) {
            fail(prec, d, "Soft limits changed during move");
        } else if (fabs(m->val - d->target) > m->mres / 2) {
            /* VAL is not what we last commanded: the user moved the setpoint,
             * so dither about the new one. Wait a cycle before commanding. */
            d->center = m->val;
            d->target = m->val;
        } else if (done && !moving) {
            lo = d->center - d->minus;
            hi = d->center + d->plus;
            if (m->limits) {
                lo = fmax(lo, m->llm);
                hi = fmin(hi, m->hlm);
            }
            if (!isfinite(lo) || !isfinite(hi) || hi - lo < m->mres) {
                fail(prec, d, "Dither range is empty");
            } else {
                d->target = d->direction > 0 ? hi : lo;
                d->direction = -d->direction;
                if (!put_double(prec, &prec->mval, d->target))
                    fail(prec, d, "Cannot command dither move");
            }
        }
    } else if (d->phase == STOPPING && done && !moving) {
        if (d->stop_sent) {
            d->stop_sent = 0;
        } else if (m->limits && (limits_invalid(m) || d->center < m->llm ||
                                 d->center > m->hlm)) {
            fail(prec, d, "Center outside soft limits");
        } else if (!put_double(prec, &prec->mval, d->center)) {
            fail(prec, d, "Cannot return to center");
        } else {
            d->phase = RETURNING;
        }
    } else if (d->phase == RETURNING && done && !moving) {
        if (!put_double(prec, &prec->mvel, d->old_velocity))
            fail(prec, d, "Cannot restore motor velocity");
        else {
            d->owns_velocity = 0;
            d->phase = IDLE;
        }
    } else if (was_fault && !enabled && done && !moving) {
        /* A fault clears once dithering is disabled and the motor has stopped */
        if (!d->owns_velocity || put_double(prec, &prec->mvel, d->old_velocity)) {
            d->owns_velocity = 0;
            d->phase = IDLE;
        }
    }

    /* While not dithering, the center simply follows the motor setpoint */
    if (d->phase == IDLE) {
        d->center = m->val;
        d->target = m->val;
    }
}

static long process(struct dbCommon *pcommon)
{
    ditherRecord *prec = (ditherRecord *)pcommon;
    Dither *d = prec->dpvt;
    Motor m;
    char old_err[sizeof(prec->err)];
    double old_cen = prec->cen, old_val = prec->val;
    int start = prec->start != 0, stop = prec->stop != 0;
    epicsEnum16 old_state = prec->state;
    unsigned short events;

    memset(&m, 0, sizeof(m));
    m.rbv = NAN;
    prec->pact = TRUE;
    if (d) {
        /* STOP wins over START. START is only accepted from Idle. */
        if (stop) {
            d->enabled = 0;
        } else if (start) {
            if (d->phase == IDLE)
                d->enabled = 1;
            else
                errlogPrintf("%s: START ignored; dithering is %s (write STOP first)\n",
                             prec->name, d->phase == FAULT ? "faulted" : "busy");
        }
    }
    prec->start = prec->stop = 0;
    if (!d) {
        recGblSetSevr(prec, STATE_ALARM, INVALID_ALARM);
    } else {
        memcpy(old_err, prec->err, sizeof(old_err));
        if (!all_links_local(prec)) {
            fail(prec, d, "All links must be local DB links");
        } else if (!read_motor(prec, &m)) {
            fail(prec, d, "Cannot read motor fields");
        } else {
            run(prec, d, &m);
        }
        prec->cen = d->center;
        prec->val = isfinite(m.rbv) ? m.rbv - d->center : NAN;
        prec->state = d->phase == DITHER ? ditherSTAT_Dithering :
            (d->phase == STOPPING || d->phase == RETURNING ?
             ditherSTAT_Returning :
             (d->phase == FAULT ? ditherSTAT_Fault : ditherSTAT_Idle));
        if (d->phase == FAULT)
            recGblSetSevr(prec, STATE_ALARM, INVALID_ALARM);
    }
    recGblGetTimeStamp(prec);

    events = recGblResetAlarms(prec);
    if (events) db_post_events(prec, &prec->val, events);
    if (start) db_post_events(prec, &prec->start, DBE_VALUE | DBE_LOG);
    if (stop) db_post_events(prec, &prec->stop, DBE_VALUE | DBE_LOG);
    if (prec->val != old_val)
        db_post_events(prec, &prec->val, DBE_VALUE | DBE_LOG);
    if (prec->state != old_state)
        db_post_events(prec, &prec->state, DBE_VALUE | DBE_LOG);
    if (prec->cen != old_cen)
        db_post_events(prec, &prec->cen, DBE_VALUE | DBE_LOG);
    if (d && memcmp(old_err, prec->err, sizeof(old_err)))
        db_post_events(prec, &prec->err, DBE_VALUE | DBE_LOG);

    recGblFwdLink(prec);
    prec->pact = FALSE;
    return 0;
}
