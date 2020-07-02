/*==============================================================================
 leaf-oscillators.h
 Created: 20 Jan 2017 12:00:58pm
 Author:  Michael R Mulshine
 ==============================================================================*/

#ifndef LEAF_OSCILLATORS_H_INCLUDED
#define LEAF_OSCILLATORS_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif
    
    //==============================================================================
    
#include "leaf-math.h"
#include "leaf-mempool.h"
#include "leaf-tables.h"
#include "leaf-filters.h"
    
    /*!
     * Header.
     * @include basic-oscillators.h
     * @example basic-oscillators.c
     * An example.
     */
    
    //==============================================================================
    
    typedef struct _tCycle
    {
        tMempool mempool;
        // Underlying phasor
        float phase;
        float inc,freq;
    } _tCycle;
    
    typedef _tCycle* tCycle;
    
    /*!
     * @defgroup tcycle tCycle
     * @ingroup oscillators
     * @brief A cycle/sine waveform oscillator. Uses wavetable synthesis.
     * @{
     */
    
    //! Initialize a tCycle to the default LEAF mempool.
    /*!
     @param osc A pointer to the tCycle to be initialized.
     */
    void    tCycle_init         (tCycle* const osc);
    
    
    //! Initialize a tCycle to a specified mempool.
    /*!
     @param osc A pointer to the tCycle to be initialized.
     @param pool A pointer to the tMempool to which the tCycle should be initialized.
     */
    void    tCycle_initToPool   (tCycle* const osc, tMempool* const pool);
    
    
    //! Free a tCycle from the default LEAF mempool.
    /*!
     @param osc A pointer to the tCycle to be freed.
     */
    void    tCycle_free         (tCycle* const osc);
    
    
    //! Tick a tCycle oscillator.
    /*!
     @param osc A pointer to the relevant tCycle.
     @return The ticked sample as a float from -1 to 1.
     */
    float   tCycle_tick         (tCycle* const osc);
    
    
    //! Set the frequency of a tCycle oscillator.
    /*!
     @param osc A pointer to the relevant tCycle.
     @param freq The frequency to set the oscillator to.
     */
    void    tCycle_setFreq      (tCycle* const osc, float freq);
    
    /*! @} */
    
    //==============================================================================
    
    /* tTriangle: Anti-aliased Triangle waveform using wavetable interpolation. Wavetables constructed from sine components. */
    typedef struct _tTriangle
    {
        tMempool mempool;
        // Underlying phasor
        float phase;
        float inc,freq;
        int oct;
        float w;
    } _tTriangle;
    
    typedef _tTriangle* tTriangle;
    
    /*!
     * @defgroup ttriangle tTriangle
     * @ingroup oscillators
     * @brief An anti-aliased triangle waveform oscillator. Uses wavetable synthesis.
     * @{
     */
    
    //! Initialize a tTriangle to the default LEAF mempool.
    /*!
     @param osc A pointer to the tTriangle to be initialized.
     */
    void    tTriangle_init          (tTriangle* const osc);
    
    
    //! Initialize a tTriangle to a specified mempool.
    /*!
     @param osc A pointer to the tTriangle to be initialized.
     @param pool A pointer to the tMempool to which the tTriangle should be initialized.
     */
    void    tTriangle_initToPool    (tTriangle* const osc, tMempool* const pool);
    
    
    //! Free a tTriangle from the default LEAF mempool.
    /*!
     @param osc A pointer to the tTriangle to be freed.
     */
    void    tTriangle_free          (tTriangle* const osc);
    
    
    //! Tick a tTriangle oscillator.
    /*!
     @param osc A pointer to the relevant tTriangle.
     @return The ticked sample as a float from -1 to 1.
     */
    float   tTriangle_tick          (tTriangle* const osc);
    
    
    //! Set the frequency of a tTriangle oscillator.
    /*!
     @param osc A pointer to the relevant tTriangle.
     @param freq The frequency to set the oscillator to.
     */
    void    tTriangle_setFreq       (tTriangle* const osc, float freq);
    
    /*! @} */
    
    //==============================================================================
    
    /* tSquare: Anti-aliased Square waveform using wavetable interpolation. Wavetables constructed from sine components. */
    typedef struct _tSquare
    {
        tMempool mempool;
        // Underlying phasor
        float phase;
        float inc,freq;
        int oct;
        float w;
    } _tSquare;
    
    typedef _tSquare* tSquare;
    
    /*!
     * @defgroup tsquare tSquare
     * @ingroup oscillators
     * @brief An anti-aliased square waveform oscillator. Uses wavetable synthesis.
     * @{
     */
    
    //! Initialize a tSquare to the default LEAF mempool.
    /*!
     @param osc A pointer to the tSquare to be initialized.
     */
    void    tSquare_init        (tSquare* const osc);
    
    
    //! Initialize a tSquare to a specified mempool.
    /*!
     @param osc A pointer to the tSquare to be initialized.
     @param pool A pointer to the tMempool to which the tSquare should be initialized.
     */
    void    tSquare_initToPool  (tSquare* const osc, tMempool* const);
    
    
    //! Free a tSquare from the default LEAF mempool.
    /*!
     @param osc A pointer to the tSquare to be freed.
     */
    void    tSquare_free        (tSquare* const osc);
    
    
    //! Tick a tSquare oscillator.
    /*!
     @param osc A pointer to the relevant tSquare.
     @return The ticked sample as a float from -1 to 1.
     */
    float   tSquare_tick        (tSquare* const osc);

    
    //! Set the frequency of a tSquare oscillator.
    /*!
     @param osc A pointer to the relevant tSquare.
     @param freq The frequency to set the oscillator to.
     */
    void    tSquare_setFreq     (tSquare* const osc, float freq);
    
    /*! @} */
    
    //==============================================================================
    
    /* tSawtooth: Anti-aliased Sawtooth waveform using wavetable interpolation. Wavetables constructed from sine components. */
    typedef struct _tSawtooth
    {
        tMempool mempool;
        // Underlying phasor
        float phase;
        float inc,freq;
        int oct;
        float w;
    } _tSawtooth;
    
    typedef _tSawtooth* tSawtooth;
    
    /*!
     * @defgroup tsawtooth tSawtooth
     * @ingroup oscillators
     * @brief An anti-aliased sawtooth waveform oscillator. Uses wavetable synthesis.
     * @{
     */
    
    //! Initialize a tSawtooth to the default LEAF mempool.
    /*!
     @param osc A pointer to the tSawtooth to be initialized.
     */
    void    tSawtooth_init          (tSawtooth* const osc);
    
    
    //! Initialize a tSawtooth to a specified mempool.
    /*!
     @param osc A pointer to the tSawtooth to be initialized.
     @param pool A pointer to the tMempool to which the tSawtooth should be initialized.
     */
    void    tSawtooth_initToPool    (tSawtooth* const osc, tMempool* const pool);
    
    
    //! Free a tSawtooth from the default LEAF mempool.
    /*!
     @param osc A pointer to the tSawtooth to be freed.
     */
    void    tSawtooth_free          (tSawtooth* const osc);
    
    
    //! Tick a tSawtooth oscillator.
    /*!
     @param osc A pointer to the relevant tSawtooth.
     @return The ticked sample as a float from -1 to 1.
     */
    float   tSawtooth_tick          (tSawtooth* const osc);
    
    
    //! Set the frequency of a tSawtooth oscillator.
    /*!
     @param osc A pointer to the relevant tSawtooth.
     @param freq The frequency to set the oscillator to.
     */
    void    tSawtooth_setFreq       (tSawtooth* const osc, float freq);
    
    /*! @} */
    
    //==============================================================================
    
    typedef struct _tSine
    {
        tMempool mempool;
        float* sine;
        int size;
        float phase;
        float inc,freq;
    } _tSine;
    
    typedef _tSine* tSine;
    
    /*!
     * @defgroup tsine tSine
     * @ingroup oscillators
     * @brief A cycle/sine waveform oscillator.
     * @{
     */
    
    //! Initialize a tSine to the default LEAF mempool.
    /*!
     @param osc A pointer to the tSine to be initialized.
     */
    void    tSine_init         (tSine* const osc, int size);
    
    
    //! Initialize a tSine to a specified mempool.
    /*!
     @param osc A pointer to the tSine to be initialized.
     @param pool A pointer to the tMempool to which the tSine should be initialized.
     */
    void    tSine_initToPool   (tSine* const osc, int size, tMempool* const pool);
    
    
    //! Free a tSine from the default LEAF mempool.
    /*!
     @param osc A pointer to the tSine to be freed.
     */
    void    tSine_free         (tSine* const osc);
    
    
    //! Tick a tSine oscillator.
    /*!
     @param osc A pointer to the relevant tSine.
     @return The ticked sample as a float from -1 to 1.
     */
    float   tSine_tick         (tSine* const osc);
    
    
    //! Set the frequency of a tSine oscillator.
    /*!
     @param osc A pointer to the relevant tSine.
     @param freq The frequency to set the oscillator to.
     */
    void    tSine_setFreq      (tSine* const osc, float freq);
    
    /*! @} */
    
    //==============================================================================
    
    /* tTri: Anti-aliased Triangle waveform using wavetable interpolation. */
    typedef struct _tTri
    {
        tMempool mempool;
        float phase;
        float inc,freq;
        float skew;
        float lastOut;
    } _tTri;
    
    typedef _tTri* tTri;
    
    /*!
     * @defgroup tTri tTri
     * @ingroup oscillators
     * @brief An anti-aliased triangle waveform oscillator.
     * @{
     */
    
    //! Initialize a tTri to the default LEAF mempool.
    /*!
     @param osc A pointer to the tTri to be initialized.
     */
    void    tTri_init          (tTri* const osc);
    
    
    //! Initialize a tTri to a specified mempool.
    /*!
     @param osc A pointer to the tTri to be initialized.
     @param pool A pointer to the tMempool to which the tTri should be initialized.
     */
    void    tTri_initToPool    (tTri* const osc, tMempool* const pool);
    
    
    //! Free a tTri from the default LEAF mempool.
    /*!
     @param osc A pointer to the tTri to be freed.
     */
    void    tTri_free          (tTri* const osc);
    
    
    //! Tick a tTri oscillator.
    /*!
     @param osc A pointer to the relevant tTri.
     @return The ticked sample as a float from -1 to 1.
     */
    float   tTri_tick          (tTri* const osc);
    
    
    //! Set the frequency of a tTri oscillator.
    /*!
     @param osc A pointer to the relevant tTriangle.
     @param freq The frequency to set the oscillator to.
     */
    void    tTri_setFreq       (tTri* const osc, float freq);
    
    void    tTri_setSkew       (tTri* const osc, float skew);
    
    /*! @} */
    
    //==============================================================================
    
    /* tPulse: Anti-aliased Square waveform. */
    typedef struct _tPulse
    {
        tMempool mempool;
        float phase;
        float inc,freq;
        float width;
    } _tPulse;
    
    typedef _tPulse* tPulse;
    
    /*!
     * @defgroup tPulse tPulse
     * @ingroup oscillators
     * @brief An anti-aliased pulse waveform oscillator with changeable pulse width.
     * @{
     */
    
    //! Initialize a tPulse to the default LEAF mempool.
    /*!
     @param osc A pointer to the tPulse to be initialized.
     */
    void    tPulse_init        (tPulse* const osc);
    
    
    //! Initialize a tPulse to a specified mempool.
    /*!
     @param osc A pointer to the tPulse to be initialized.
     @param pool A pointer to the tMempool to which the tPulse should be initialized.
     */
    void    tPulse_initToPool  (tPulse* const osc, tMempool* const);
    
    
    //! Free a tPulse from the default LEAF mempool.
    /*!
     @param osc A pointer to the tPulse to be freed.
     */
    void    tPulse_free        (tPulse* const osc);
    
    
    //! Tick a tPulse oscillator.
    /*!
     @param osc A pointer to the relevant tPulse.
     @return The ticked sample as a float from -1 to 1.
     */
    float   tPulse_tick        (tPulse* const osc);
    
    
    //! Set the frequency of a tPulse oscillator.
    /*!
     @param osc A pointer to the relevant tPulse.
     @param freq The frequency to set the oscillator to.
     */
    void    tPulse_setFreq     (tPulse* const osc, float freq);
    
    void    tPulse_setWidth    (tPulse* const osc, float width);
    
    /*! @} */
    
    //==============================================================================
    
    /* tSawtooth: Anti-aliased Sawtooth waveform. */
    typedef struct _tSaw
    {
        tMempool mempool;
        float phase;
        float inc,freq;
    } _tSaw;
    
    typedef _tSaw* tSaw;
    
    /*!
     * @defgroup tsaw tSaw
     * @ingroup oscillators
     * @brief An anti-aliased sawtooth waveform oscillator. Uses wavetable synthesis.
     * @{
     */
    
    //! Initialize a tSaw to the default LEAF mempool.
    /*!
     @param osc A pointer to the tSaw to be initialized.
     */
    void    tSaw_init          (tSaw* const osc);
    
    
    //! Initialize a tSaw to a specified mempool.
    /*!
     @param osc A pointer to the tSaw to be initialized.
     @param pool A pointer to the tMempool to which the tSaw should be initialized.
     */
    void    tSaw_initToPool    (tSaw* const osc, tMempool* const pool);
    
    
    //! Free a tSaw from the default LEAF mempool.
    /*!
     @param osc A pointer to the tSaw to be freed.
     */
    void    tSaw_free          (tSaw* const osc);
    
    
    //! Tick a tSaw oscillator.
    /*!
     @param osc A pointer to the relevant tSaw.
     @return The ticked sample as a float from -1 to 1.
     */
    float   tSaw_tick          (tSaw* const osc);
    
    
    //! Set the frequency of a tSaw oscillator.
    /*!
     @param osc A pointer to the relevant tSaw.
     @param freq The frequency to set the oscillator to.
     */
    void    tSaw_setFreq       (tSaw* const osc, float freq);
    
    /*! @} */
    
    //==============================================================================
    
    /* tPhasor: Aliasing phasor [0.0, 1.0) */
    typedef struct _tPhasor
    {
        tMempool mempool;
        float phase;
        float inc,freq;
        uint8_t phaseDidReset;
    } _tPhasor;
    
    typedef _tPhasor* tPhasor;
    
    /*!
     * @defgroup tphasor Phasor
     * @ingroup oscillators
     * @brief An aliasing phasor.
     * @{
     */
    
    //! Initialize a tPhasor to the default LEAF mempool.
    /*!
     @param osc A pointer to the tPhasor to be initialized.
     */
    void    tPhasor_init        (tPhasor* const osc);
    
    
    //! Initialize a tPhasor to a specified mempool.
    /*!
     @param osc A pointer to the tPhasor to be initialized.
     @param pool A pointer to the tMempool to which the tPhasor should be initialized.
     */
    void    tPhasor_initToPool  (tPhasor* const osc, tMempool* const);
    
    
    //! Free a tPhasor from the default LEAF mempool.
    /*!
     @param osc A pointer to the tPhasor to be freed.
     */
    void    tPhasor_free        (tPhasor* const osc);
    

    //! Tick a tPhasor oscillator.
    /*!
     @param osc A pointer to the relevant tPhasor.
     @return The ticked sample as a float from 0 to 1.
     */
    float   tPhasor_tick        (tPhasor* const osc);
    
    
    //! Set the frequency of a tPhasor oscillator.
    /*!
     @param osc A pointer to the relevant tPhasor.
     @param freq The frequency to set the oscillator to.
     */
    void    tPhasor_setFreq     (tPhasor* const osc, float freq);
    
    /*! @} */
    
    //==============================================================================
    
    /*!
     * @defgroup tnoise tNoise
     * @ingroup oscillators
     * @brief A noise generator.
     * @{
     */
    
    /* tNoise. WhiteNoise, PinkNoise. */
    /*!
     * Noise types
     */
    enum NoiseType
    {
        WhiteNoise, //!< White noise. Full spectrum.
        PinkNoise, //!< Pink noise. Inverse frequency-proportional spectrum.
        NoiseTypeNil,
    };
    
    typedef enum NoiseType NoiseType;
    
    typedef struct _tNoise
    {
        tMempool mempool;
        NoiseType type;
        float pinkb0, pinkb1, pinkb2;
        float(*rand)(void);
    } _tNoise;
    
    typedef _tNoise* tNoise;
    
    //! Initialize a tNoise to the default LEAF mempool.
    /*!
     @param osc A pointer to the tNoise to be initialized.
     */
    void    tNoise_init         (tNoise* const noise, NoiseType type);
    
    
    //! Initialize a tNoise to a specified mempool.
    /*!
     @param osc A pointer to the tNoise to be initialized.
     @param pool A pointer to the tMempool to which the tNoise should be initialized.
     */
    void    tNoise_initToPool   (tNoise* const noise, NoiseType type, tMempool* const);
    
    
    //! Free a tNoise from the default LEAF mempool.
    /*!
     @param osc A pointer to the tNoise to be freed.
     */
    void    tNoise_free         (tNoise* const noise);
    
    
    //! Tick a tNoise oscillator.
    /*!
     @param osc A pointer to the relevant tNoise.
     @return The ticked sample as a float from -1 to 1.
     */
    float   tNoise_tick         (tNoise* const noise);
    
    /*! @} */
    
    //==============================================================================
    
    /*!
     * @defgroup tneuron tNeuron
     * @ingroup oscillators
     * @brief A model of a neuron, adapted to act as an oscillator.
     * @{
     */
    
    /*!
     * Neuron modes
     */
    enum NeuronMode
    {
        NeuronNormal, //!< Normal operation
        NeuronTanh, //!< Tanh voltage shaping
        NeuronAaltoShaper, //!< Aalto voltage shaping
        NeuronModeNil
    };
    typedef enum NeuronMode NeuronMode;
    
    typedef struct _tNeuron
    {
        tMempool mempool;
        
        tPoleZero f;
        
        NeuronMode mode;
        
        float voltage, current;
        float timeStep;
        
        float alpha[3];
        float beta[3];
        float rate[3];
        float V[3];
        float P[3];
        float gK, gN, gL, C;
    } _tNeuron;
    
    typedef _tNeuron* tNeuron;
    
    //! Initialize a tNeuron to the default LEAF mempool.
    /*!
     @param neuron A pointer to the tNeuron to be initialized.
     */
    void    tNeuron_init        (tNeuron* const neuron);
    
    
    //! Initialize a tNeuron to a specified mempool.
    /*!
     @param neuron A pointer to the tNeuron to be initialized.
     @param pool A pointer to the tMempool to which the tNeuron should be initialized.
     */
    void    tNeuron_initToPool  (tNeuron* const neuron, tMempool* const pool);
    
    
    //! Free a tNeuron from the default LEAF mempool.
    /*!
     @param neuron A pointer to the tNeuron to be freed.
     */
    void    tNeuron_free        (tNeuron* const neuron);
    
    
    //! Reset the neuron model.
    /*!
     @param neuron A pointer to the relevant tNeuron.
     */
    void    tNeuron_reset       (tNeuron* const neuron);
    
    
    //! Tick a tNeuron oscillator.
    /*!
     @param neuron A pointer to the relevant tNeuron.
     @return The ticked sample as a float from -1 to 1.
     */
    float   tNeuron_tick        (tNeuron* const neuron);
    
    
    //! Set the tNeuron shaping mode.
    /*!
     @param neuron A pointer to the relevant tNeuron.
     @param mode The mode to set the tNeuron to.
     */
    void    tNeuron_setMode     (tNeuron* const neuron, NeuronMode mode);
    
    
    //! Set the current.
    /*!
     @param neuron A pointer to the relevant tNeuron.
     @param current The new current.
     */
    void    tNeuron_setCurrent  (tNeuron* const neuron, float current);
    
    
    //! Set the potassium value.
    /*!
     @param neuron A pointer to the relevant tNeuron.
     @param K The new potassium.
     */
    void    tNeuron_setK        (tNeuron* const neuron, float K);
    
    
    //! Set the chloride value.
    /*!
     @param neuron A pointer to the relevant tNeuron.
     @param L The new chloride value.
     */
    void    tNeuron_setL        (tNeuron* const neuron, float L);
    
    
    //! Set the sodium value.
    /*!
     @param neuron A pointer to the relevant tNeuron.
     @param N The new sodium value.
     */
    void    tNeuron_setN        (tNeuron* const neuron, float N);
    
    
    //! Set the calcium value.
    /*!
     @param neuron A pointer to the relevant tNeuron.
     @param C The new calcium.
     */
    void    tNeuron_setC        (tNeuron* const neuron, float C);
    
    
    //! Set the V1 value.
    /*!
     @param neuron A pointer to the relevant tNeuron.
     @param V1 The new V1.
     */
    void    tNeuron_setV1       (tNeuron* const neuron, float V1);
    
    
    //! Set the V2 value.
    /*!
     @param neuron A pointer to the relevant tNeuron.
     @param V2 The new V2.
     */
    void    tNeuron_setV2       (tNeuron* const neuron, float V2);
    
    
    //! Set the V3 value.
    /*!
     @param neuron A pointer to the relevant tNeuron.
     @param V3 The new V3.
     */
    void    tNeuron_setV3       (tNeuron* const neuron, float V3);
    
    
    //! Set the time step.
    /*!
     @param neuron A pointer to the relevant tNeuron.
     @param timestep The new time step
     */
    void    tNeuron_setTimeStep (tNeuron* const neuron, float timestep);
    
    /*! @} */
    
    //==============================================================================
    
#define FILLEN 256
    
    typedef struct _tMBPulse
    {
        tMempool mempool;
        float    out;
        float    amp;
        float    last_amp;
        float    freq;
        float    waveform;    // duty cycle, must be in [-1, 1]
        float    syncin;
        float    syncout;
        float   _p, _w, _b, _x, _z;
        float   _f [FILLEN + STEP_DD_PULSE_LENGTH];
        int     _j, _k;
        bool    _init;
    } _tMBPulse;
    
    typedef _tMBPulse* tMBPulse;
    
    void tMBPulse_init(tMBPulse* const osc);
    void tMBPulse_initToPool(tMBPulse* const osc, tMempool* const pool);
    void tMBPulse_free(tMBPulse* const osc);
    
    float tMBPulse_tick(tMBPulse* const osc);
    void tMBPulse_setFreq(tMBPulse* const osc, float f);
    void tMBPulse_setWidth(tMBPulse* const osc, float w);
    void tMBPulse_syncIn(tMBPulse* const osc, float sync);
    float tMBPulse_syncOut(tMBPulse* const osc);
    
    
    typedef struct _tMBTriangle
    {
        tMempool mempool;
        float    out;
        float    amp;
        float    last_amp;
        float    freq;
        float    waveform;    // duty cycle, must be in [-1, 1]
        float    syncin;
        float    syncout;
        float   _p, _w, _b, _z;
        float   _f [FILLEN + LONGEST_DD_PULSE_LENGTH];
        int     _j, _k;
        bool    _init;
    } _tMBTriangle;
    
    typedef _tMBTriangle* tMBTriangle;
    
    void tMBTriangle_init(tMBTriangle* const osc);
    void tMBTriangle_initToPool(tMBTriangle* const osc, tMempool* const pool);
    void tMBTriangle_free(tMBTriangle* const osc);
    
    float tMBTriangle_tick(tMBTriangle* const osc);
    void tMBTriangle_setFreq(tMBTriangle* const osc, float f);
    void tMBTriangle_setWidth(tMBTriangle* const osc, float w);
    void tMBTriangle_syncIn(tMBTriangle* const osc, float sync);
    float tMBTriangle_syncOut(tMBTriangle* const osc);
    
    
    
    typedef struct _tMBSaw
    {
        tMempool mempool;
        float    out;
        float    amp;
        float    last_amp;
        float    freq;
        float    syncin;
        float    syncout;
        float   _p, _w, _z;
        float   _f [FILLEN + STEP_DD_PULSE_LENGTH];
        int     _j;
        bool    _init;
    } _tMBSaw;
    
    typedef _tMBSaw* tMBSaw;
    
    void tMBSaw_init(tMBSaw* const osc);
    void tMBSaw_initToPool(tMBSaw* const osc, tMempool* const pool);
    void tMBSaw_free(tMBSaw* const osc);
    
    float tMBSaw_tick(tMBSaw* const osc);
    void tMBSaw_setFreq(tMBSaw* const osc, float f);
    void tMBSaw_syncIn(tMBSaw* const osc, float sync);
    float tMBSaw_syncOut(tMBSaw* const osc);
    
    
    
    
#ifdef __cplusplus
}
#endif

#endif  // LEAF_OSCILLATORS_H_INCLUDED

//==============================================================================

